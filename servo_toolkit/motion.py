"""One-shot, expiring C001 bench motion entry; chat authorization is still required."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import time
from uuid import uuid4

from . import servo

PLAN_TTL_S = 300
SCOPES = ("center", "center-jog50")


def source_sha256() -> str:
    """Content hash, excluding mutable records and interpreter/test caches."""
    root = Path(__file__).resolve().parent
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root)
        if (set(rel.parts) & {"records", "__pycache__", ".pytest_cache"}
                or path.suffix == ".pyc" or not path.is_file()):
            continue
        digest.update(rel.as_posix().encode() + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def _device(port: str) -> dict:
    path = Path(port)
    if not path.is_absolute():
        raise servo.ServoError("受控入口要求绝对串口路径")
    real = path.resolve(strict=True)
    info = real.stat()
    if not stat.S_ISCHR(info.st_mode):
        raise servo.ServoError("指定路径不是串口字符设备")
    if not os.access(real, os.R_OK | os.W_OK):
        raise servo.ServoError("串口权限未开放；请用户自行处理，不使用 sudo")
    return {"path": str(real), "dev": info.st_dev, "ino": info.st_ino,
            "rdev": info.st_rdev, "ctime_ns": info.st_ctime_ns}


def _digest(plan: dict) -> str:
    return hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode()).hexdigest()


def _same(expected: servo.Snapshot, current: servo.Snapshot) -> None:
    servo._center_preconditions(current)
    if any(getattr(expected, key) != getattr(current, key) for key in servo.MOVE_CENTER_FIELDS):
        raise servo.ServoError("本轮计划的舵机 ID/配置已变化；停止并重新检查")
    if abs(expected.position - current.position) > servo.MOVE_CENTER_TOLERANCE:
        raise servo.ServoError("本轮计划后位置已变化；停止并重新检查")


def _centered(snapshot: servo.Snapshot) -> None:
    servo._center_preconditions(snapshot)
    if abs(snapshot.position - servo.CENTER) > servo.MOVE_CENTER_TOLERANCE:
        raise servo.ServoError("最终位置不在 2048 ±3 中位窗内；不自动回中")


def _inputs(port: str, sid: int, scope: str, baud: int) -> None:
    servo.valid_id(sid)
    if scope not in SCOPES or baud != servo.BAUD:
        raise servo.ServoError("受控入口仅允许 1 Mbps 与 center/center-jog50 两种动作范围")
    if not isinstance(port, str) or not Path(port).is_absolute():
        raise servo.ServoError("受控入口要求绝对串口路径")


def _plans(records: Path) -> Path:
    return records / ".motion-plans"


def _save_new(path: Path, data: dict) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as file:
        json.dump(data, file, ensure_ascii=False, sort_keys=True)
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())
    servo._sync_dir(path.parent)


def prepare(*, port: str, sid: int, scope: str, baud: int, records: Path) -> dict:
    _inputs(port, sid, scope, baud)
    device, source = _device(port), source_sha256()
    with servo.connect(port, baud=baud) as bus:
        before = bus.inspect(sid)
        servo._center_preconditions(before)
        if scope == "center-jog50" and not (before.angle_min <= 1995 and before.angle_max >= 2101):
            raise servo.ServoError("限位未覆盖中位窗 ±50，停止")
        _same(before, bus.inspect(sid))
    if _device(port) != device or source_sha256() != source:
        raise servo.ServoError("准备期间设备节点或源码变化，停止")
    now = time.time()
    plan = {"version": 1, "plan_id": uuid4().hex, "port": port, "device": device,
            "sid": sid, "scope": scope, "created_at": now, "expires_at": now + PLAN_TTL_S,
            "source": source, "snapshot": asdict(before)}
    if not records.parent.is_dir():
        raise servo.ServoError("记录目录上级不存在")
    records.mkdir(exist_ok=True)
    servo._sync_dir(records.parent)
    folder = _plans(records)
    folder.mkdir(mode=0o700, exist_ok=True)
    servo._sync_dir(records)
    _save_new(folder / (plan["plan_id"] + ".json"), plan)
    return {"state": "prepared_read_only", "plan_id": plan["plan_id"],
            "approve_sha256": _digest(plan), "scope": scope, "id": sid,
            "expires_at": plan["expires_at"], "position": before.position,
            "snapshot": asdict(before), "plan_record": str(folder / (plan["plan_id"] + ".json")),
            "notice": "未执行动作；须在聊天中取得本轮现场授权，许可不能跨重连/换舵机沿用"}


def _load(records: Path, plan_id: str, approval: str, port: str,
          sid: int, scope: str, baud: int) -> dict:
    _inputs(port, sid, scope, baud)
    if (not isinstance(plan_id, str) or not re.fullmatch(r"[0-9a-f]{32}", plan_id)
            or not isinstance(approval, str) or not re.fullmatch(r"[0-9a-f]{64}", approval)):
        raise servo.ServoError("缺失或无效的本轮 plan/approve")
    path = _plans(records) / (plan_id + ".json")
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, "r", encoding="utf-8") as file:
        info = os.fstat(file.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 32768:
            raise servo.ServoError("计划文件类型/大小无效")
        plan = json.load(file)
    if not isinstance(plan, dict) or _digest(plan) != approval:
        raise servo.ServoError("计划内容与本轮批准摘要不符")
    if (plan.get("version"), plan.get("plan_id"), plan.get("port"), plan.get("sid"), plan.get("scope")) != (
            1, plan_id, port, sid, scope):
        raise servo.ServoError("计划与本轮串口、ID 或动作范围不符")
    created, expires = plan.get("created_at"), plan.get("expires_at")
    if (type(created) not in (float, int) or type(expires) not in (float, int)
            or not math.isfinite(created) or not math.isfinite(expires)
            or expires - created != PLAN_TTL_S or not created <= time.time() < expires):
        raise servo.ServoError("本轮计划已过期或时间无效；不得重用")
    if (_plans(records) / (plan_id + ".used")).exists():
        raise servo.ServoError("本轮计划已消费（含失败），不得重用或自动重试")
    if plan.get("source") != source_sha256() or plan.get("device") != _device(port):
        raise servo.ServoError("源码/设备节点已变化，可能已重连；本轮计划失效")
    before = servo.Snapshot(**plan["snapshot"])
    if before.id != sid:
        raise servo.ServoError("计划快照 ID 不符")
    servo._center_preconditions(before)
    return plan


class _ScopedBus:
    """Only the existing motion functions receive this bus; cleanup is never blocked by expiry."""
    def __init__(self, bus, permit):
        self._bus, self._permit = bus, permit

    def __getattr__(self, name):
        return getattr(self._bus, name)

    def write1(self, sid, address, value):
        # Permit expiry/identity changes must not prevent the existing cleanup attempt.
        if (sid, address, value) != (self._permit.plan["sid"], 40, 0):
            self._permit.live()
            if (sid, address, value) != (self._permit.plan["sid"], 40, 1):
                raise servo.ServoError("受控动作禁止其他寄存器写入")
        return self._bus.write1(sid, address, value)

    def write2(self, sid, address, value):
        self._permit.live()
        if (sid, address) != (self._permit.plan["sid"], 42):
            raise servo.ServoError("受控动作只允许已有目标写入路径")
        return self._bus.write2(sid, address, value)


def _check_lifetime(plan, deadline):
    if (not plan["created_at"] <= time.time() < plan["expires_at"]
            or time.monotonic() >= deadline):
        raise servo.ServoError("本轮动作许可已过期或时间回拨，停止；只保留既有卸力清理")


class _MotionPermit:
    def __init__(self, plan, bus, *, deadline):
        self.plan = plan
        self.deadline = deadline  # anchored at run entry, never renewed after blocking I/O
        self.bus = _ScopedBus(bus, self)
        self.next_action = "center"
        self.claimed = set()
        self.expected = servo.Snapshot(**plan["snapshot"])

    def live(self):
        _check_lifetime(self.plan, self.deadline)
        if _device(self.plan["port"]) != self.plan["device"]:
            raise servo.ServoError("设备节点变化/重连，本轮许可作废")

    def claim(self, bus, port, expected, action):
        self.live()
        if (bus is not self.bus or port != self.plan["port"] or expected != self.expected
                or action != self.next_action or action in self.claimed
                or source_sha256() != self.plan["source"]):
            raise servo.ServoError("受控许可的连接、快照、源码或动作顺序不符")
        self.next_action = None  # consumes the stage before any bus/record access
        self.claimed.add(action)

    def allow_jog(self, snapshot):
        if (self.plan["scope"] != "center-jog50" or self.next_action is not None
                or self.claimed != {"center"}):
            raise servo.ServoError("本轮没有后续点动授权")
        _centered(snapshot)
        self.expected, self.next_action = snapshot, "jog50"


def run(*, port: str, sid: int, scope: str, baud: int, records: Path,
        plan_id: str, approval: str, report: dict) -> dict:
    # Capture both clocks before plan loading, fsync, or a potentially blocking connect.
    # Reading monotonic first makes scheduling delays shorten, never renew, the permit.
    started_mono, started_wall = time.monotonic(), time.time()
    plan = _load(records, plan_id, approval, port, sid, scope, baud)
    deadline = started_mono + min(PLAN_TTL_S, max(0, plan["expires_at"] - started_wall))
    before = servo.Snapshot(**plan["snapshot"])
    # Atomic O_EXCL plus directory fsync: every attempt, even connect failure, burns the plan.
    _save_new(_plans(records) / (plan_id + ".used"), {"started_at": time.time(), "scope": scope})
    report["plan_record"] = str(_plans(records) / (plan_id + ".json"))
    report["stage"] = "connect"
    # fsync and binding checks consume the same lifetime as connection and motion.
    if _device(port) != plan["device"] or source_sha256() != plan["source"]:
        raise servo.ServoError("消费计划期间设备/源码变化；未连接，不重试")
    try:
        _check_lifetime(plan, deadline)
    except servo.ServoError as exc:
        raise servo.ServoError(f"{exc}；未连接，不重试") from exc
    with servo.connect(port, baud=baud) as raw_bus:
        permit = _MotionPermit(plan, raw_bus, deadline=deadline)
        permit.live()
        _same(before, raw_bus.inspect(sid))
        report["stage"] = "center"
        center = servo.move_center(permit.bus, port=port, expected=before,
                                   confirmation=servo.CONFIRM_MOVE_CENTER, records=records, _permit=permit)
        observed = servo.Snapshot(**center["snapshot"])
        _centered(observed)
        _same(observed, raw_bus.inspect(sid))
        report["center"] = {"status": "passed", "result": center}
        if scope == "center-jog50":
            report["stage"] = "jog50"
            permit.allow_jog(observed)
            result = servo.jog_test(permit.bus, port=port, expected=observed, amplitude=50,
                                    confirmation=servo.CONFIRM_JOG, records=records, _permit=permit)
            report["jog50"] = {"status": "passed", "result": result}
        report["stage"] = "final_inspect"
        permit.live()
        final = raw_bus.inspect(sid)
        _centered(final)
        if any(getattr(before, key) != getattr(final, key) for key in servo.MOVE_CENTER_FIELDS):
            raise servo.ServoError("最终配置与本轮计划不同，停止")
        report["final_inspect"] = {"status": "passed", "snapshot": asdict(final)}
    report.update(exit_code=0, stage="complete", state="passed")
    return report


def motion_command(args, *, records: Path) -> int:
    report = {"state": "stopped", "exit_code": 1, "stage": "validation",
              "center": {"status": "not_run"}, "jog50": {"status": "not_run"},
              "final_inspect": {"status": "not_run"}}
    try:
        if args.command == "motion-prepare":
            report = prepare(port=args.port, sid=args.id, scope=args.scope, baud=args.baud, records=records)
            report["exit_code"] = 0
        else:
            run(port=args.port, sid=args.id, scope=args.scope, baud=args.baud, records=records,
                plan_id=args.plan, approval=args.approve, report=report)
    except (servo.ServoError, OSError, ValueError, KeyError, TypeError, ImportError, KeyboardInterrupt) as exc:
        stage = report.get("stage")
        if stage in ("center", "jog50", "final_inspect"):
            report[stage] = {"status": "failed_or_uncertain"}
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["notice"] = ("已停止，不自动重试。若动作可能开始：出力状态未知；"
                            "请立即切断舵机外部 7.5 V 并确认失电，不只拔 USB，不抓轴。"
                            "换舵机/双路断电重连后旧动作授权失效；新流程须重新检查与授权。")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report["exit_code"]
