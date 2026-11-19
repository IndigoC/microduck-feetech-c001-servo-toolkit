"""HD-1910-C001 单颗台架调试；运动命令独立门禁，默认关闭。

move-center 驱动到已有 2048；calibrate-center 重定义当前位置，二者不等价。
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
from uuid import uuid4

BAUD = 1_000_000
CENTER = 2048
TOLERANCE = 4  # 位置读回的暂定容差，单位为编码器计数，不代表机械精度。
CONFIRM_ID = "确认单颗修改ID"
CONFIRM_CENTER = "确认单颗校准中位"
CONFIRM_JOG = "确认单颗点动测试"

# 点动只写 RAM 寄存器（目标 42 / 扭矩 40），不碰任何 EEPROM 参数。
JOG_MAX_AMPLITUDE = 200   # 单边幅度上限，计数；200 ≈ 17.6°
JOG_MIN_AMPLITUDE = 30    # 单边幅度下限，计数；30 ≈ 2.6°。低于此值无法从位置反馈分辨是否真的动
JOG_TOLERANCE = 12        # 到位判定窗，计数；12 ≈ 1.05°，宽于齿轮虚位 ≤0.5°；工程值，待实机核实
JOG_TIMEOUT_S = 2.0       # 单个目标的到位超时
JOG_SETTLE_S = 0.2        # 卸力后等待静止再事后核对

# 点动会在实物上开启扭矩。安全设计、异常卸力语义与独立复审未通过前保持关闭，
# 也不提供命令行开关；下层 SDK 仍可写，禁止绕过。
JOG_ARMED = False

# 驱动到已有中位：仅命令层步长/节拍约束，不是固件限速，也不保证实际速度/冲击。
CONFIRM_MOVE_CENTER = "确认单颗驱动到中位"
MOVE_CENTER_ARMED = False  # 与 JOG_ARMED 无关；代码授权不等于实机动作授权。
MOVE_CENTER_TOLERANCE = 3  # 只读中位流程沿用的暂定读回窗，不是机械装配公差。
MOVE_CENTER_STEP = 32      # 新目标距本步新鲜位置至多 32 counts（约 2.8°）。
MOVE_CENTER_INTERVAL_S = 1.0  # 每步确认停稳后至少等待此值再做步间检查；不会追赶时间表。
MOVE_CENTER_STEP_TIMEOUT_S = 1.0
MOVE_CENTER_TOTAL_TIMEOUT_S = 180.0  # 覆盖逐步运动及每步等待；不是连续持力安全保证。
MOVE_CENTER_FIELDS = ("id", "ping_model_number", "firmware_major", "firmware_minor",
                      "model_low", "model_high", "endian", "baud_index", "secondary_id",
                      "response_level", "angle_min", "angle_max", "phase", "resolution",
                      "offset_raw", "mode", "lock")


class ServoError(Exception):
    """安全前提不满足、通信失败或写入结果不确定。"""


class CommunicationError(ServoError):
    def __init__(self, operation: str, result: int, status: int):
        super().__init__(f"{operation}：通信结果={result}，舵机状态=0x{status:02X}")
        self.result = result
        self.status = status


def valid_id(sid: int) -> int:
    # 254 为广播，255 为帧头；两者都不能传入任何操作。
    if isinstance(sid, bool) or not isinstance(sid, int) or not 0 <= sid <= 253:
        raise ServoError("舵机 ID 必须是 0–253 的整数，不允许广播")
    return sid


def signed_magnitude(raw: int) -> int:
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


def valid_offset(raw: int) -> bool:
    # HLS 表：BIT15 为符号，幅值至多 4095，其余保留位不得置位。
    return isinstance(raw, int) and 0 <= raw <= 0xFFFF and not (raw & 0x7000)


@dataclass(frozen=True)
class Snapshot:
    id: int
    ping_model_number: int
    firmware_major: int
    firmware_minor: int
    model_low: int
    model_high: int
    endian: int
    baud_index: int
    secondary_id: int
    response_level: int
    angle_min: int
    angle_max: int
    phase: int
    resolution: int
    offset_raw: int
    offset: int
    mode: int
    torque: int
    goal_raw: int
    lock: int
    position: int
    speed: int
    voltage_v: float
    temperature_c: int
    status: int
    moving: int


class HlsBus:
    """在官方 hls 协议对象上做严格结果检查；可注入假 SDK 进行无硬件测试。"""

    def __init__(self, handler, success: int, timeout: int):
        self.handler, self.success, self.timeout = handler, success, timeout

    def _result(self, name: str, result: int, status: int) -> None:
        if result != self.success or status != 0:
            raise CommunicationError(name, result, status)

    def ping(self, sid: int) -> None:
        valid_id(sid)
        _unused, result, status = self.handler.ping(sid)
        self._result(f"ID {sid} PING", result, status)

    def read1(self, sid: int, address: int) -> int:
        valid_id(sid)
        value, result, status = self.handler.read1ByteTxRx(sid, address)
        self._result(f"ID {sid} 读地址 {address}", result, status)
        return value

    def read2(self, sid: int, address: int) -> int:
        valid_id(sid)
        value, result, status = self.handler.read2ByteTxRx(sid, address)
        self._result(f"ID {sid} 读地址 {address}", result, status)
        return value

    def write1(self, sid: int, address: int, value: int) -> None:
        valid_id(sid)
        result, status = self.handler.write1ByteTxRx(sid, address, value)
        self._result(f"ID {sid} 写地址 {address}", result, status)

    def write2(self, sid: int, address: int, value: int) -> None:
        valid_id(sid)
        result, status = self.handler.write2ByteTxRx(sid, address, value)
        self._result(f"ID {sid} 写地址 {address}", result, status)

    def calibrate(self, sid: int) -> None:
        valid_id(sid)
        result, status = self.handler.reOfsCal(sid, CENTER)
        self._result(f"ID {sid} 位置校准", result, status)

    def probe(self, sid: int) -> bool:
        """False 仅表示本次没观察到应答，并不证明无设备或没有副 ID。"""
        try:
            observed = self.read1(sid, 5)
        except CommunicationError as exc:
            if exc.result == self.timeout and exc.status == 0:
                return False
            raise
        if observed != sid:
            raise ServoError(f"向 ID {sid} 查询时读到主 ID {observed}，停止")
        return True

    def inspect(self, sid: int) -> Snapshot:
        valid_id(sid)
        self.ping(sid)
        read1 = lambda addr: self.read1(sid, addr)
        read2 = lambda addr: self.read2(sid, addr)
        endian = read1(2)
        if endian != 0:
            raise ServoError(f"字节序标志为 {endian}，本工具只支持已假设的小端；不解释多字节数据")
        model = read2(3)
        actual = read1(5)
        if actual != sid:
            raise ServoError(f"查询 ID {sid} 读回主 ID {actual}，停止")
        offset_raw = read2(31)
        return Snapshot(
            id=actual, ping_model_number=model, firmware_major=read1(0), firmware_minor=read1(1),
            model_low=read1(3), model_high=read1(4), endian=endian,
            baud_index=read1(6), secondary_id=read1(7), response_level=read1(8),
            angle_min=read2(9), angle_max=read2(11), phase=read1(18), resolution=read1(30),
            offset_raw=offset_raw, offset=signed_magnitude(offset_raw),
            mode=read1(33), torque=read1(40), goal_raw=read2(42), lock=read1(55),
            position=signed_magnitude(read2(56)), speed=signed_magnitude(read2(58)),
            voltage_v=read1(62) / 10, temperature_c=read1(63),
            status=read1(65), moving=read1(66),
        )


@contextmanager
def connect(port_name: str, baud: int = BAUD):
    """按操作者指定的当前速率连接；不扫描，也不更改舵机波特率。"""
    if not port_name:
        raise ServoError("必须明确指定串口")
    if baud not in (1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400):
        raise ServoError("当前串口速率不在候选支持列表")
    try:
        import serial
        from .vendor.scservo_sdk import (  # 导入时不打开串口
            PortHandler, COMM_SUCCESS, COMM_RX_TIMEOUT,
        )
        from .strict_sdk import StrictHls
    except ImportError as exc:
        raise ServoError("缺少 pyserial；请先阅读 README，安装依赖须另行确认") from exc
    port = PortHandler(port_name)
    try:
        # 与 pyserial 的合作进程独占；仍不能阻止不遵守锁的外部程序。
        # 打开串口可能改变 DTR/RTS，必须在现场确认、经授权之后操作。
        port.ser = serial.Serial(port=port_name, baudrate=baud, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 timeout=0, write_timeout=1.0, exclusive=True)
        port.is_open = True
        yield HlsBus(StrictHls(port), COMM_SUCCESS, COMM_RX_TIMEOUT)
    finally:
        if port.is_open:
            port.closePort()


# 已由实机只读核对过的型号读数：(ping_model_number, model_low, model_high)。
# 2026-11-15 台架单颗 HD-1910-C001 实测：ping 7946、型号高/低字节 31/10、固件 3.46。
# 只承认白名单内的读数，未列出的型号一律拒绝；没有命令行绕过开关。
VERIFIED_PROFILES = {(7946, 10, 31): "HD-1910-C001"}


def _require_validated_profile(s: Snapshot) -> None:
    """型号读数必须命中实机只读核对过的白名单，才允许写入。"""
    key = (s.ping_model_number, s.model_low, s.model_high)
    if key not in VERIFIED_PROFILES:
        raise ServoError(f"型号读数 {key} 不在已实机核对的 profile 白名单内，禁止实机写入")


def _preconditions(s: Snapshot, *, calibration: bool, allowed_torque: tuple[int, ...] = (0,)) -> None:
    if s.endian != 0 or s.baud_index != 0 or s.response_level != 1:
        raise ServoError("字节序、波特率或应答级别不符合候选 HLS 假设，禁止写入")
    if s.status != 0:
        raise ServoError(f"舵机状态寄存器为 0x{s.status:02X}，请先人工排查")
    if s.lock != 1:
        raise ServoError("写入前 EPROM 必须处于锁定状态；请先人工排查")
    if s.torque not in allowed_torque or s.moving != 0 or s.speed != 0:
        raise ServoError(f"单颗台架状态不符：torque={s.torque}, moving={s.moving}, speed={s.speed}；"
                         f"要求静止且 torque 在 {allowed_torque} 内")
    if not 4.0 <= s.voltage_v <= 8.4:
        raise ServoError(f"读回电压 {s.voltage_v} V 超出 C001 的 4–8.4 V 范围")
    if not valid_offset(s.offset_raw):
        raise ServoError(f"位置偏移原始值 0x{s.offset_raw:04X} 含保留位，停止")
    if calibration:
        if (s.firmware_major, s.firmware_minor) < (3, 43):
            raise ServoError("候选 HLS 固件低于 3.43，不发送 0x0B")
        if (s.mode != 0 or s.phase & 0x10 or s.resolution != 1
                or not 0 <= s.angle_min < s.angle_max <= 4095
                or not s.angle_min <= CENTER <= s.angle_max):
            raise ServoError("运行模式、分辨率或角度限制不符合单圈位置校准的保守前提")
        if not 0 <= s.position <= 4095:
            raise ServoError("当前位置不在单圈 0–4095 范围")


def _fresh_snapshot(bus: HlsBus, expected: Snapshot, *, calibration: bool) -> Snapshot:
    """交互确认之后重读，避免确认期间状态/位置发生变化。"""
    current = bus.inspect(expected.id)
    _require_validated_profile(current)
    _preconditions(current, calibration=calibration)
    identity = ("ping_model_number", "firmware_major", "firmware_minor", "model_low", "model_high",
                "secondary_id", "phase", "offset_raw")
    if any(getattr(expected, field) != getattr(current, field) for field in identity):
        raise ServoError("确认期间舵机信息或位置偏移变化，停止并重新检查")
    if calibration and abs(current.position - expected.position) > TOLERANCE:
        raise ServoError("确认期间位置发生变化；请固定参考位后重新检查")
    return current


def _sync_dir(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _record_start(records: Path, action: str, port: str, before: Snapshot, target: int) -> Path:
    """任何写入前先写并同步旧状态及目录；失败则不做写入。"""
    if not records.parent.is_dir():
        raise ServoError("记录目录的上级目录必须已存在")
    records.mkdir(exist_ok=True)
    _sync_dir(records.parent)  # 包括刚创建 records/ 的情况。
    name = f"{datetime.now(timezone.utc):%Y%m%dT%H%M%S.%fZ}-{action}-{uuid4().hex[:8]}.jsonl"
    path = records / name
    row = {"event": "before", "utc": datetime.now(timezone.utc).isoformat(),
           "action": action, "port": port, "target": target, "snapshot": asdict(before)}
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as file:
        file.write(json.dumps(row, ensure_ascii=False) + "\n")
        file.flush()
        os.fsync(file.fileno())
    _sync_dir(records)  # 新文件目录项也需要同步。
    return path


def _record_result(path: Path, event: str, data: dict) -> None:
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps({"event": event, "utc": datetime.now(timezone.utc).isoformat(),
                               **data}, ensure_ascii=False) + "\n")
        file.flush()
        os.fsync(file.fileno())


def _unlocked_operation(bus: HlsBus, old_id: int, new_id: int | None, operation,
                        record_path: Path):
    """解锁并执行，最后尽力在可确定的实际 ID 上重锁并回读。

    ID 写入可能已生效但应答超时，因此调用者在发 ID 写入前设置 attempt 标记。
    无法确定唯一地址时绝不盲写；结果明确标记为不确定。
    """
    id_attempted = [False]
    unlock_attempted = False
    result = None
    failure = None
    try:
        _record_result(record_path, "before_unlock", {"id": old_id})
        unlock_attempted = True  # 发送异常也可能已送达。
        bus.write1(old_id, 55, 0)
        if bus.read1(old_id, 55) != 0:
            raise ServoError("解锁未能回读确认，不执行操作")
        _record_result(record_path, "unlock_confirmed", {"id": old_id})
        result = operation(id_attempted)
    except BaseException as exc:
        failure = exc
    if not unlock_attempted:
        raise ServoError(f"解锁前记录失败，未开始写入：{failure}") from failure
    port = getattr(bus.handler, "portHandler", None)
    if getattr(port, "_microduck_tainted", False):
        raise ServoError(_uncertain_power_off_message(f"通信异常：{failure}")) from failure
    cleanup_failure = None
    try:
        if id_attempted[0]:
            if result is None or new_id is None:
                raise ServoError("ID 写入后尚未可靠读回新 ID，无法确定重锁地址")
            lock_id = new_id
        else:
            lock_id = old_id
        # 记录失败也不能阻止为安全目的尽力重新上锁。
        log_failure = None
        try:
            _record_result(record_path, "before_relock", {"id": lock_id})
        except BaseException as exc:
            # 即使在记录阶段 Ctrl-C，也尽力做一次正常重锁（链路仍须可信）。
            log_failure = exc
        try:
            bus.write1(lock_id, 55, 1)
        except CommunicationError as exc:
            if exc.result != bus.timeout or exc.status != 0:
                raise
            # StrictHls 已将超时连接标为 tainted；不要再发一包只读造成碰撞。
            if getattr(port, "_microduck_tainted", False):
                raise ServoError(f"ID {lock_id} 重锁 ACK 超时，通信链路已污染") from exc
            # 仅非 tainted 的可控假总线允许一次只读确认；不能据此证明掉电保存。
            if bus.read1(lock_id, 55) != 1:
                raise ServoError(f"ID {lock_id} 重锁 ACK 超时且锁标志非 1") from exc
        else:
            if bus.read1(lock_id, 55) != 1:
                raise ServoError(f"ID {lock_id} 的重锁未能回读确认")
        if log_failure:
            raise ServoError(f"ID {lock_id} 回读锁标志为 1，但重锁前记录失败：{log_failure}")
    except BaseException as exc:
        cleanup_failure = exc
    if cleanup_failure:
        raise ServoError(_uncertain_power_off_message(
            f"操作异常/结果：{failure or '主体已返回'}；重锁失败或状态未知：{cleanup_failure}"
        )) from (failure or cleanup_failure)
    try:
        _record_result(record_path, "relock_confirmed", {"id": lock_id, "lock": 1})
    except OSError as exc:
        raise ServoError(f"已回读 ID {lock_id} 锁标志为 1，但写入记录失败：{exc}；"
                         "主体结果及断电持久性仍需核查") from exc
    if failure:
        raise ServoError(f"操作失败或结果不确定：{failure}；已回读确认 ID {lock_id} 重锁，"
                         "但不能据此判断写入是否生效。请勿自动重试，先只读排查。") from failure
    return result


def _execute_recorded(path: Path, operation):
    try:
        result = operation()
    except BaseException as exc:
        try:
            _record_result(path, "error", {"error": f"{type(exc).__name__}: {exc}"})
        except OSError as log_exc:
            raise ServoError(f"{exc}；结果记录追加失败：{log_exc}；原始记录：{path}") from exc
        raise ServoError(f"{exc}；操作记录：{path}") from exc
    try:
        _record_result(path, "result", result)
    except OSError as exc:
        raise ServoError(f"操作结束，但结果记录写入失败：{exc}；原始记录：{path}；结果：{result}") from exc
    return result


def change_id(bus: HlsBus, *, port: str, expected: Snapshot, new_id: int,
              confirmation: str, records: Path) -> dict:
    """只操作一颗未装机、扭矩已关闭的舵机；需要用户确认并留存写前记录。"""
    if confirmation != CONFIRM_ID:
        raise ServoError("未给出准确的单颗改 ID 确认语")
    valid_id(new_id)
    old_id = valid_id(expected.id)
    if new_id == old_id:
        raise ServoError("新旧 ID 相同，无需写入")
    current = _fresh_snapshot(bus, expected, calibration=False)
    # 不预探测新 ID：超时无法证明空闲，也会污染连接；必须物理上仅接一颗。
    path = _record_start(records, "change-id", port, current, new_id)

    def run():
        def write(attempted):
            _record_result(path, "before_id_write", {"old_id": old_id, "new_id": new_id})
            attempted[0] = True  # 必须在发送前设置，超时也可能已经改 ID。
            bus.write1(old_id, 5, new_id)
            if bus.read1(new_id, 5) != new_id:
                raise ServoError("写后主 ID 未回读为目标值，结果未知")
            return {"old_id": old_id, "new_id": new_id}
        result = _unlocked_operation(bus, old_id, new_id, write, path)
        # 在重锁之后再回读整个状态；断电持久性必须由用户后续单独验证。
        after = bus.inspect(new_id)
        _record_result(path, "after_read", {"snapshot": asdict(after)})
        _preconditions(after, calibration=False)
        if (after.ping_model_number, after.firmware_major, after.firmware_minor, after.offset_raw) != (
            current.ping_model_number, current.firmware_major, current.firmware_minor, current.offset_raw
        ):
            raise ServoError("写 ID 后的型号、固件或位置偏移与写前不同，停止")
        result.update(snapshot=asdict(after),
                      persistence="未验证；断电重连后只读核对", record=str(path))
        return result
    return _execute_recorded(path, run)


def calibrate_center(bus: HlsBus, *, port: str, expected: Snapshot,
                     confirmation: str, records: Path) -> dict:
    """把已确认的静止机械参考位置重标为 2048；不会运动、上力或恢复旧目标。"""
    if confirmation != CONFIRM_CENTER:
        raise ServoError("未给出准确的单颗中位校准确认语")
    current = _fresh_snapshot(bus, expected, calibration=True)
    path = _record_start(records, "calibrate-center", port, current, CENTER)

    def run():
        def calibrate(_attempted):
            # 最后再检查扭矩/位置；不替用户卸力，避免松轴和自动上力。
            if bus.read1(current.id, 40) != 0:
                raise ServoError("校准前扭矩已不是关闭状态，停止")
            if bus.read1(current.id, 65) != 0 or bus.read1(current.id, 66) != 0:
                raise ServoError("校准前舵机状态异常或正在运动，停止")
            if signed_magnitude(bus.read2(current.id, 58)) != 0:
                raise ServoError("校准前速度不为零，停止")
            pos = signed_magnitude(bus.read2(current.id, 56))
            if abs(pos - current.position) > TOLERANCE:
                raise ServoError("校准前位置发生变化，停止")
            _record_result(path, "before_calibrate", {"id": current.id, "target": CENTER})
            bus.calibrate(current.id)
            time.sleep(0.1)  # 等待校准后寄存器更新，不控制关节。
            return None
        _unlocked_operation(bus, current.id, None, calibrate, path)
        after = bus.inspect(current.id)
        # 即使接下来的验证失败，也保留校准后的实际观测。
        _record_result(path, "after_read", {"snapshot": asdict(after)})
        _preconditions(after, calibration=True)
        if (after.ping_model_number, after.firmware_major, after.firmware_minor, after.mode) != (
            current.ping_model_number, current.firmware_major, current.firmware_minor, current.mode
        ):
            raise ServoError("校准后的型号、固件或模式发生变化，停止")
        if abs(after.position - CENTER) > TOLERANCE:
            raise ServoError(f"位置回读 {after.position} 不在中位 {CENTER} ± {TOLERANCE} 内")
        if after.offset_raw == current.offset_raw:
            if abs(current.position - CENTER) > TOLERANCE:
                raise ServoError("偏移未变，不能确认 0x0B 生效")
            state = "位置原本接近中位且偏移未变，无法证明校准指令是否生效"
            verified = False
        else:
            state = "位置与偏移发生变化，但偏移方向/环绕语义尚未验证，不能判定校准成功"
            verified = False
        return {"state": state, "effect_observed": verified, "before": asdict(current), "after": asdict(after),
                "persistence": "未验证；断电重连后只读核对", "record": str(path)}
    return _execute_recorded(path, run)


def track_position(bus: HlsBus, servo_id: int, *, target: int = CENTER,
                   tolerance: int = 3, interval: float = 0.04,
                   settle_reads: int = 5) -> int:
    """只读观察位置：不上力、不写任何寄存器，不再默认要求手动反驱。

    通信异常直接上报、不重试，也不用旧读数掩盖总线问题。
    1 计数 = 0.088°；默认读回窗 3 计数，不代表机械装配公差。
    连续 settle_reads 次读数不变才报告稳定，不构成装配姿态验收。
    """
    valid_id(servo_id)
    if not 0 <= target <= 4095:
        raise ServoError("目标位置必须在 0–4095")
    last = None
    stable = 0
    try:
        while True:
            pos = signed_magnitude(bus.read2(servo_id, 56))
            if not 0 <= pos <= 4095:
                raise ServoError(f"位置读数 {pos} 不在单圈 0–4095，停止")
            # 单圈环绕：取最短方向的角度差，避免手转过 0 时读数误导。
            delta = (target - pos + 2048) % 4096 - 2048
            stable = stable + 1 if last is not None and pos == last else 0
            last = pos
            settled = stable >= settle_reads
            index = round(pos / 4095 * 20)
            bar = "".join("#" if i == index else "-" for i in range(21))
            if abs(delta) <= tolerance and settled:
                print(f"\rID {servo_id}  pos {pos:4d}  [{bar}]  "
                      f"稳定在 {target} ± {tolerance} 计数            ", end="", flush=True)
                print(f"\n已稳定：pos={pos}  Δ={delta:+d}（{delta * 360 / 4096:+.2f}°）；"
                      "仅报告当前读数稳定；装配参考姿态与后续位置须另行核对。")
                return 0
            mark = f"Δ {delta:+5d} ({delta * 360 / 4096:+6.2f}°)" + ("  静止" if settled else "")
            print(f"\rID {servo_id}  pos {pos:4d}  [{bar}]  {mark}  ", end="", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\n已停止跟踪；最后位置 {last}。")
        return 3


def _goto(bus: HlsBus, sid: int, target: int) -> int:
    """写目标并等待到位；状态异常或超时立即抛出，由调用方负责卸力。

    位置用绝对差比较，不做 %4096 单圈折叠：C001 控制表 bit4=1 为全角度反馈，
    折叠会把“差一整圈”当成到位。读数不在 0–4095 或未停稳都视为异常。
    """
    bus.write2(sid, 42, target & 0xFFFF)
    deadline = time.monotonic() + JOG_TIMEOUT_S
    last = None
    settled = 0
    while time.monotonic() < deadline:
        status = bus.read1(sid, 65)
        if status != 0:
            raise ServoError(f"运动中出现舵机状态 0x{status:02X}，立即停止")
        pos = signed_magnitude(bus.read2(sid, 56))
        if not 0 <= pos <= 4095:
            raise ServoError(f"位置读数 {pos} 不在单圈 0–4095，反馈异常或已多圈，停止")
        moving = bus.read1(sid, 66)
        last = pos
        if abs(pos - target) <= JOG_TOLERANCE and moving == 0:
            settled += 1
            if settled >= 2:
                return pos
        else:
            settled = 0
        time.sleep(0.02)
    raise ServoError(f"目标 {target} 在 {JOG_TIMEOUT_S}s 内未到位或未停稳（最后读数 {last}）")


def _motion_guard(bus: HlsBus, before: Snapshot, tolerance: int, *, goal_synced: bool,
                  center: bool = True, center_window: bool = False) -> int:
    """默认保留完整中位前提；jog 显式用原通用前提，写后仅放宽 torque 为 0/1。"""
    observed = bus.inspect(before.id)
    allowed_torque = (0, 1) if goal_synced else (0,)
    if center:
        _center_preconditions(observed, allowed_torque=allowed_torque)
    else:
        _require_validated_profile(observed)
        _preconditions(observed, calibration=False, allowed_torque=allowed_torque)
    if any(getattr(before, name) != getattr(observed, name) for name in MOVE_CENTER_FIELDS):
        raise ServoError("运动前配置变化，停止")
    if center_window and abs(observed.position - CENTER) > MOVE_CENTER_TOLERANCE:
        raise ServoError("受控点动 guard 已偏离绝对中位窗，停止")
    if abs(observed.position - before.position) > tolerance:
        raise ServoError("上力前目标/位置变化，位置偏离起始，停止")
    if goal_synced and observed.goal_raw != before.position:
        raise ServoError("目标同步回读与起始位置不符，停止")
    return observed.torque


def jog_test(bus: HlsBus, *, port: str, expected: Snapshot, amplitude: int,
             confirmation: str, records: Path, _permit=None) -> dict:
    """单颗台架点动：先把目标同步到当前位置，再小幅往返，结束后卸力并校验参数未变。

    只写 RAM 寄存器 42（目标位置）与 40（扭矩），不写 ID（5）、波特率（6）、
    相位（18）、偏移（31）、模式（33），所以不影响已完成的 ID、波特率与中位预处理；
    结束后重新读快照逐项比对，任何一项变化都会报错。断电后 40/42 复位，不写入 EEPROM。
    """
    if confirmation != CONFIRM_JOG:
        raise ServoError("未给出准确的点动测试确认语")
    if _permit is None:
        _require_jog_armed()
    else:
        _accept_motion_permit(_permit, bus, port, expected, "jog50")
        if amplitude != 50 or isinstance(amplitude, bool):
            raise ServoError("受控入口仅允许点动 50 counts")
    if not isinstance(amplitude, int) or not JOG_MIN_AMPLITUDE <= amplitude <= JOG_MAX_AMPLITUDE:
        raise ServoError(f"点动幅度必须是 {JOG_MIN_AMPLITUDE}–{JOG_MAX_AMPLITUDE} 计数"
                         f"（约 {JOG_MIN_AMPLITUDE * 360 / 4096:.1f}°–{JOG_MAX_AMPLITUDE * 360 / 4096:.1f}°）；"
                         "低于下限时位置反馈分辨不出是否真的运动")
    current = _fresh_snapshot(bus, expected, calibration=False)
    if _permit is not None:
        _center_preconditions(current)
        if (abs(current.position - CENTER) > MOVE_CENTER_TOLERANCE
                or abs(current.position - expected.position) > MOVE_CENTER_TOLERANCE):
            raise ServoError("受控点动前已偏离中位窗或起始位置，停止")
        if any(getattr(expected, key) != getattr(current, key) for key in MOVE_CENTER_FIELDS):
            raise ServoError("受控点动前配置变化，停止")
    if current.mode != 4:
        raise ServoError(f"点动只针对出厂的位置闭环模式；当前 mode={current.mode}")
    base = current.position
    if not 0 <= base <= 4095:
        raise ServoError(f"起始位置 {base} 不在单圈 0–4095，反馈异常或已多圈，不点动")
    for edge in (base + amplitude, base - amplitude):
        if not current.angle_min <= edge <= current.angle_max:
            raise ServoError(f"点动目标 {edge} 超出限位 {current.angle_min}–{current.angle_max}")
    path = _record_start(records, "jog", port, current, base)

    power_uncertain = False

    def run():
        nonlocal power_uncertain
        power_attempted = False
        failure = None
        events = []
        try:
            # 写目标也会使能：日志先于最终卸力 guard，从首次写目标起负责卸力。
            _record_result(path, "pre_torque_guard", {"expected_goal": base})
            controlled = _permit is not None
            guard_tolerance = MOVE_CENTER_TOLERANCE if controlled else JOG_TOLERANCE
            _motion_guard(bus, current, guard_tolerance, goal_synced=False,
                          center=controlled, center_window=controlled)
            power_attempted = power_uncertain = True
            bus.write2(current.id, 42, base)
            torque = _motion_guard(bus, current, guard_tolerance, goal_synced=True,
                                   center=controlled, center_window=controlled)
            events.append(("goal_synced", {"goal": base, "torque_observed": torque}))
            if torque == 0:
                bus.write1(current.id, 40, 1)
                if bus.read1(current.id, 40) != 1:
                    raise ServoError("扭矩开启未能回读确认")
            events.append(("torque_on", {"id": current.id, "implicit": torque == 1}))
            # 带力阶段仅内存记录；卸力确认后才同步到磁盘。
            last = base
            for target in (base + amplitude, base, base - amplitude, base):
                pos = _goto(bus, current.id, target)
                travel = pos - last
                expected = target - last
                events.append(("jog_step",
                               {"target": target, "position": pos,
                                "travel": travel, "expected": expected}))
                # 此差等于 pos-target，重复到位窗检查，不是独立方向或轨迹验证。
                if abs(travel - expected) > JOG_TOLERANCE:
                    raise ServoError(
                        f"到目标 {target} 的实际位移 {travel} 与期望 {expected} 不符，"
                        "可能未真正驱动、方向相反或反馈异常")
                last = pos
        except BaseException as exc:
            failure = exc
        # 4) 无论成败都尝试卸力；链路已不可信时不盲发，改为要求断电。
        if power_attempted:
            handler = getattr(bus, "handler", None)
            port = getattr(handler, "portHandler", None)
            if getattr(port, "_microduck_tainted", False):
                raise ServoError(_torque_may_be_on_message(
                    f"上力或点动期间通信异常：{failure}")) from failure
            try:
                bus.write1(current.id, 40, 0)
                if bus.read1(current.id, 40) != 0:
                    raise ServoError("扭矩关闭未能回读确认")
                power_uncertain = False
                events.append(("torque_off", {"id": current.id}))
            except BaseException as exc:
                raise ServoError(_torque_may_be_on_message(
                    f"卸力失败：{exc}")) from exc
        for event, data in events:
            _record_result(path, event, data)
        if failure is not None:
            state = ("已尝试卸力并回读确认，但扭矩状态最终以现场失电为准"
                     if power_attempted else "未发送目标或上力指令")
            raise ServoError(f"点动未完成：{failure}；{state}") from failure
        # 5) 事后核对：参数不变、停在原位附近、扭矩已关
        time.sleep(JOG_SETTLE_S)
        after = bus.inspect(current.id)
        if after.torque != 0:
            power_uncertain = True
            raise ServoError(_torque_may_be_on_message("点动结束后扭矩非 0"))
        _record_result(path, "after_read", {"snapshot": asdict(after)})
        if after.moving != 0 or after.speed != 0:
            raise ServoError(f"点动后尚未确认静止：moving={after.moving}，speed={after.speed}；"
                             "不判定完成，请勿接触输出轴或开始装配")
        if after.status != 0:
            raise ServoError(f"点动后状态寄存器 0x{after.status:02X}，请人工排查")
        identity = ("id", "ping_model_number", "firmware_major", "firmware_minor",
                    "baud_index", "phase", "offset_raw", "mode", "lock",
                    "angle_min", "angle_max", "secondary_id", "response_level", "resolution")
        changed = {name: (getattr(current, name), getattr(after, name))
                   for name in identity if getattr(current, name) != getattr(after, name)}
        if changed:
            raise ServoError(f"点动后参数发生变化，需人工核对：{changed}")
        if not 0 <= after.position <= 4095:
            raise ServoError(f"点动后位置 {after.position} 不在单圈 0–4095，反馈异常或已多圈，"
                             "不能判定回位")
        drift = abs(after.position - base)
        if drift > JOG_TOLERANCE:
            raise ServoError(f"点动后位置 {after.position} 偏离起始 {base} 达 {drift} 计数；"
                             "扭矩已关闭，可手动摆回中位后重新检查")
        return {"base": base, "final_position": after.position, "drift": drift,
                "amplitude": amplitude, "torque": after.torque,
                "checked_unchanged": list(identity),
                "snapshot": asdict(after), "record": str(path)}

    # 未确认卸力时直接警告断电，不先同步错误日志。
    try:
        result = run()
    except BaseException as exc:
        link = getattr(getattr(bus, "handler", None), "portHandler", None)
        if power_uncertain or getattr(link, "_microduck_tainted", False):
            detail = str(exc)
            if "请立即切断舵机外部电源" not in detail:
                detail = _torque_may_be_on_message(detail)
            raise ServoError(f"{detail}；原始记录可能不完整：{path}") from exc
        _record_result(path, "error", {"error": f"{type(exc).__name__}: {exc}"})
        raise ServoError(f"{exc}；操作记录：{path}") from exc
    _record_result(path, "result", result)
    return result


def _center_preconditions(s: Snapshot, *, allowed_torque: tuple[int, ...] = (0,)) -> None:
    _require_validated_profile(s)
    _preconditions(s, calibration=False, allowed_torque=allowed_torque)
    # 新运动流程只覆盖这批已读回的固件/模式；不改模式或相位来绕过限制。
    if ((s.firmware_major, s.firmware_minor) != (3, 46) or s.mode != 4
            or s.phase != 52 or s.resolution != 1):
        raise ServoError("驱动到中位仅针对已读回的 fw=3.46、mode=4、phase=52、resolution=1")
    if not 0 <= s.angle_min < s.angle_max <= 4095:
        raise ServoError("驱动到中位要求有效单圈限位 0–4095")
    if not (s.angle_min <= s.position <= s.angle_max
            and s.angle_min <= CENTER <= s.angle_max):
        raise ServoError("当前位置或中位 2048 不在单圈限位内，不执行")


def _center_feedback(bus: HlsBus, sid: int, low: int, high: int) -> int:
    """运动中的一次只读反馈；非原子采样不能替代现场旁护。"""
    if bus.read1(sid, 65) != 0:
        raise ServoError("驱动到中位期间出现状态报警，停止")
    if bus.read1(sid, 40) != 1:
        raise ServoError("驱动到中位期间扭矩状态非 1，停止")
    pos = signed_magnitude(bus.read2(sid, 56))
    if not low <= pos <= high:
        raise ServoError(f"位置读数 {pos} 不在单圈限位 {low}–{high}，停止")
    return pos


def _center_steps(bus: HlsBus, s: Snapshot, deadline: float, steps: list[dict]) -> None:
    """逐步到位后才发下一目标；不动/滞后不会累积远处目标。"""
    last_confirmed = None
    for _ in range(4096):  # 有限目标数兜底；通常不足 70 步，不自动重试。
        if last_confirmed is not None:
            time.sleep(max(0, last_confirmed + MOVE_CENTER_INTERVAL_S - time.monotonic()))
        if time.monotonic() >= deadline:
            raise ServoError("驱动到中位超过总超时，停止")
        previous_target = steps[-1]["target"] if steps else s.position
        if signed_magnitude(bus.read2(s.id, 42)) != previous_target:
            raise ServoError("步间目标变化，停止")
        if bus.read1(s.id, 66) != 0 or signed_magnitude(bus.read2(s.id, 58)) != 0:
            raise ServoError("步间尚未静止，停止")
        start = _center_feedback(bus, s.id, s.angle_min, s.angle_max)
        if time.monotonic() >= deadline:
            raise ServoError("驱动到中位超过总超时，停止")
        if abs(start - previous_target) > MOVE_CENTER_TOLERANCE:
            raise ServoError("步间位置偏离上一目标，停止")
        if previous_target == CENTER and abs(start - CENTER) <= MOVE_CENTER_TOLERANCE:
            return
        distance = CENTER - start
        target = start + (1 if distance > 0 else -1) * min(MOVE_CENTER_STEP, abs(distance))
        # 目标基于刚读的位置；到这里没有日志 I/O，不用上一轮目标累计行程。
        write_started = time.monotonic()
        bus.write2(s.id, 42, target)
        # 单步/总超时从写前计，包含发送/ACK；步间等待另从停稳确认后计。
        step_deadline = min(deadline, write_started + MOVE_CENTER_STEP_TIMEOUT_S)
        settled = 0
        while True:
            if time.monotonic() >= step_deadline:
                raise ServoError(f"目标 {target} 未到位/未停稳或超过总超时，停止")
            if signed_magnitude(bus.read2(s.id, 42)) != target:
                raise ServoError("运动目标回读变化，停止")
            pos = _center_feedback(bus, s.id, s.angle_min, s.angle_max)
            moving = bus.read1(s.id, 66)
            speed = signed_magnitude(bus.read2(s.id, 58))
            if time.monotonic() >= step_deadline:
                raise ServoError(f"目标 {target} 未到位/未停稳或超过总超时，停止")
            if not min(start, target) - MOVE_CENTER_TOLERANCE <= pos <= max(start, target) + MOVE_CENTER_TOLERANCE:
                raise ServoError("位置偏离本步行程，疑似反向或过冲，停止")
            settled = settled + 1 if (abs(pos - target) <= MOVE_CENTER_TOLERANCE
                                      and moving == 0 and speed == 0) else 0
            if settled >= 2:
                last_confirmed = time.monotonic()
                steps.append({"start": start, "target": target, "position": pos})
                break
            time.sleep(0.02)
    raise ServoError("驱动到中位目标数超限，停止")


def move_center(bus: HlsBus, *, port: str, expected: Snapshot,
                confirmation: str, records: Path, _permit=None) -> dict:
    """移动到已有 2048 并卸力；仅写 RAM 40/42，不重新校零，不改 EEPROM。"""
    # 必须在任何总线访问、记录或写入之前拒绝；独立于 jog 门禁。
    if _permit is not None:
        _accept_motion_permit(_permit, bus, port, expected, "center")
    elif not MOVE_CENTER_ARMED:
        raise ServoError("驱动到中位默认关闭；当前版本禁止实机上力")
    if confirmation != CONFIRM_MOVE_CENTER:
        raise ServoError("未给出准确的单颗驱动到中位确认语")
    current = bus.inspect(valid_id(expected.id))
    _center_preconditions(current)
    if any(getattr(expected, name) != getattr(current, name) for name in MOVE_CENTER_FIELDS):
        raise ServoError("确认期间配置变化，请重新检查")
    if abs(expected.position - current.position) > MOVE_CENTER_TOLERANCE:
        raise ServoError("确认期间位置变化，请重新检查")
    if abs(current.position - CENTER) <= MOVE_CENTER_TOLERANCE:
        return {"state": "当前位置已在中位读回窗内，仅只读确认，未上力/未写入",
                "target": CENTER, "tolerance": MOVE_CENTER_TOLERANCE,
                "final_position": current.position, "torque": current.torque,
                "snapshot": asdict(current)}
    path = _record_start(records, "move-center", port, current, CENTER)
    power_uncertain = False

    def run():
        nonlocal power_uncertain
        power_attempted = False
        failure = None
        steps = []
        try:
            # 日志和最终卸力 guard 在首次目标写入之前；写目标即可能上力。
            _record_result(path, "pre_torque_guard", {"expected_goal": current.position})
            _motion_guard(bus, current, MOVE_CENTER_TOLERANCE, goal_synced=False)
            deadline = time.monotonic() + MOVE_CENTER_TOTAL_TIMEOUT_S
            power_attempted = power_uncertain = True  # 首个目标 ACK 丢失也可能已上力。
            bus.write2(current.id, 42, current.position)
            torque = _motion_guard(bus, current, MOVE_CENTER_TOLERANCE, goal_synced=True)
            if time.monotonic() >= deadline:
                raise ServoError("驱动到中位超过总超时，停止")
            if torque == 0:
                bus.write1(current.id, 40, 1)
                if bus.read1(current.id, 40) != 1:
                    raise ServoError("扭矩开启未能回读确认")
            _center_steps(bus, current, deadline, steps)
        except BaseException as exc:
            failure = exc
        # 清理之前不写日志；taint 后不再发包，异常卸力不作保证。
        link = getattr(getattr(bus, "handler", None), "portHandler", None)
        if getattr(link, "_microduck_tainted", False):
            raise ServoError(_torque_may_be_on_message(f"驱动到中位链路不可信：{failure}")) from failure
        if power_attempted:
            try:
                bus.write1(current.id, 40, 0)
                if bus.read1(current.id, 40) != 0:
                    raise ServoError("扭矩关闭未能回读确认")
                power_uncertain = False
            except BaseException as exc:
                raise ServoError(_torque_may_be_on_message(f"驱动到中位卸力失败：{exc}")) from exc
        if failure is not None:
            _record_result(path, "stopped", {"steps": steps, "torque_off_observed": power_attempted})
            state = "已回读扭矩关闭" if power_attempted else "未发送目标或上力指令"
            raise ServoError(f"驱动到中位未完成：{failure}；{state}；不自动重试") from failure
        time.sleep(JOG_SETTLE_S)
        after = bus.inspect(current.id)
        if after.torque != 0:
            power_uncertain = True
            raise ServoError(_torque_may_be_on_message("最终快照扭矩非 0"))
        # 已回读卸力后才落盘；卡死/失联仍无法保证软件卸力或记录完整。
        _record_result(path, "after_read", {"steps": steps, "snapshot": asdict(after)})
        _center_preconditions(after)  # 包括 torque/speed/moving/status=0。
        changed = [name for name in MOVE_CENTER_FIELDS
                   if getattr(current, name) != getattr(after, name)]
        if changed:
            raise ServoError(f"驱动到中位后配置变化：{changed}")
        if after.goal_raw != CENTER:
            # 最后一个小步允许反馈在窗内，但目标仍须明确为 2048。
            raise ServoError(f"最终目标回读 {after.goal_raw} 不是 2048，不能判定完成")
        if abs(after.position - CENTER) > MOVE_CENTER_TOLERANCE:
            raise ServoError(f"卸力后位置 {after.position} 偏离 2048 ± {MOVE_CENTER_TOLERANCE}，不能判定完成")
        return {"target": CENTER, "tolerance": MOVE_CENTER_TOLERANCE,
                "final_position": after.position, "torque": after.torque,
                "steps": len(steps), "checked_unchanged": list(MOVE_CENTER_FIELDS),
                "snapshot": asdict(after), "record": str(path)}

    # 未确认卸力时警告直接上抛，不先做可能阻塞的落盘；不复用通用记录包装。
    try:
        result = run()
    except BaseException as exc:
        link = getattr(getattr(bus, "handler", None), "portHandler", None)
        if power_uncertain or getattr(link, "_microduck_tainted", False):
            detail = str(exc)
            if "请立即切断舵机外部电源" not in detail:
                detail = _torque_may_be_on_message(detail)
            raise ServoError(f"{detail}；原始记录可能不完整：{path}") from exc
        _record_result(path, "error", {"error": f"{type(exc).__name__}: {exc}"})
        raise ServoError(f"{exc}；操作记录：{path}") from exc
    _record_result(path, "result", result)
    return result


def _accept_motion_permit(permit, bus, port, expected, action) -> None:
    # 仅受控入口产生的一次性、绑定连接的许可；不修改四个默认 False。
    from .motion import _MotionPermit
    if not isinstance(permit, _MotionPermit):
        raise ServoError("无效的受控动作许可")
    permit.claim(bus, port, expected, action)


def _require_jog_armed() -> None:
    """点动默认关闭：具备上力能力，未获独立复审放行前不得实机执行。"""
    if not JOG_ARMED:
        raise ServoError("点动测试尚未通过安全设计与独立复审；当前版本禁止实机上力")


def _torque_may_be_on_message(detail: str) -> str:
    return (f"{detail}；扭矩可能仍处于开启状态，目标位置也可能继续被执行。"
            "请立即切断舵机外部电源并确认实际失电，不要带电插拔；"
            "重新上电与只读核查须另行确认。")


def _uncertain_power_off_message(detail: str) -> str:
    return (f"{detail}；锁状态未知，已停止，不再发送指令。"
            "在单颗、无负载且能安全断电时，请人工关闭外部电源并确认舵机实际失电；"
            "不要带电插拔。重新上电和只读核查须另行确认。")
