"""命令行入口：python -m servo_toolkit --help"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from .servo import (CONFIRM_CENTER, CONFIRM_ID, CONFIRM_JOG, CONFIRM_MOVE_CENTER,
                    ServoError, calibrate_center, change_id, connect, jog_test,
                    move_center, track_position, valid_id)

# 写入已对实机核对过的型号开放；真正把关的是 servo.py 的型号白名单、
# 写前状态前提与写前记录；仅 CLI set-id 免手工确认。改波特率与点动仍单独禁用。
WRITES_ENABLED = True
# 点动会开扭矩：安全设计与独立复审未通过前在连接前直接拒绝（与 set-baud 同路径）。
JOG_ENABLED = False
MOVE_CENTER_ENABLED = False  # 独立门禁；连接前拒绝，不随 jog 开关放行。
RECORDS = Path(__file__).resolve().parent / "records"


def _id(value: str) -> int:
    try:
        return valid_id(int(value))
    except (ValueError, ServoError) as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _show(value):
    print(json.dumps(value, ensure_ascii=False, indent=2))


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="HD-1910-C001 单颗桌面调试（尚未实机验证）")
    root.add_argument("--port", required=True, help="明确指定串口，例如 /dev/ttyACM0")
    root.add_argument("--baud", type=int, default=1_000_000,
                      choices=(1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400),
                      help="已知的舵机当前速率（默认 1000000）；只改变本次电脑通信速率")
    commands = root.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="只读读取一颗舵机状态")
    inspect.add_argument("--id", type=_id, required=True)
    track = commands.add_parser("track", help="只读观察位置，不驱动，也不要求手拨")
    track.add_argument("--id", type=_id, required=True)
    ident = commands.add_parser("set-id", help="修改单颗舵机 ID（直接执行，不要求 TTY 或手输短语）")
    ident.add_argument("--id", type=_id, required=True, help="当前 ID")
    ident.add_argument("--new-id", type=_id, required=True, help="新 ID")
    jog = commands.add_parser("jog", help="单颗点动测试：小幅往返后回原位并卸力")
    jog.add_argument("--id", type=_id, required=True)
    jog.add_argument("--amplitude", type=int, default=100,
                     help="单边幅度，计数（默认 100 ≈ 8.8°，上限 200）")
    move = commands.add_parser("move-center", help="驱动到已有中位 2048 并卸力（默认关闭）")
    move.add_argument("--id", type=_id, required=True)
    center = commands.add_parser("calibrate-center", help="当前位置校准为 2048（需现场交互确认）")
    center.add_argument("--id", type=_id, required=True)
    baud = commands.add_parser("set-baud", help="设置舵机波特率（尚未实现、禁止使用）")
    baud.add_argument("--id", type=_id, required=True)
    for name, help_text in (("motion-prepare", "只读检查并生成本轮限时动作计划，不执行动作"),
                            ("motion-run", "执行聊天中已获现场授权的一次性动作计划")):
        motion = commands.add_parser(name, help=help_text)
        motion.add_argument("--id", type=_id, required=True)
        motion.add_argument("--scope", choices=("center", "center-jog50"), required=True)
        if name == "motion-run":
            motion.add_argument("--plan", required=True, help="本轮 motion-prepare 返回的 plan_id")
            motion.add_argument("--approve", required=True, help="本轮计划的 approve_sha256；不替代现场授权")
    return root


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    if args.command in ("motion-prepare", "motion-run"):
        if not WRITES_ENABLED:
            print("受控动作入口未开放，不会打开串口。", file=sys.stderr)
            return 2
        from .motion import motion_command
        return motion_command(args, records=RECORDS)
    if args.command == "set-baud":
        print("修改舵机波特率尚未实现；需先核实 C001 固件和对应寄存器表。", file=sys.stderr)
        return 2
    if args.command == "jog" and not JOG_ENABLED:
        print("点动测试尚未通过安全设计与独立复审：当前版本不会打开串口，也不会开启扭矩。",
              file=sys.stderr)
        return 2
    if args.command == "move-center" and not MOVE_CENTER_ENABLED:
        print("驱动到中位默认关闭：当前版本不会打开串口，也不会开启扭矩。", file=sys.stderr)
        return 2
    if args.command == "track":
        # 纯读路径：不进写入门禁，不需要交互终端，也不会发任何写指令。
        try:
            with connect(args.port, baud=args.baud) as bus:
                return track_position(bus, args.id)
        except (ServoError, OSError, ImportError, KeyboardInterrupt) as exc:
            print(f"\n已停止：{type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    if args.command != "inspect" and not WRITES_ENABLED:
        print("写入功能暂未开放：型号、固件、控制表和校准方式尚需确认与审核。", file=sys.stderr)
        return 2
    if args.command not in ("inspect", "track", "set-id") and (not sys.stdin.isatty()
                                                                 or not sys.stdout.isatty()):
        print("拒绝写入：必须由现场操作者在交互终端核对并确认。", file=sys.stderr)
        return 2
    try:
        with connect(args.port, baud=args.baud) as bus:
            before = bus.inspect(args.id)
            print("当前只读快照（候选 HLS 表，未经实机验证）：")
            _show(asdict(before))
            if args.command == "inspect":
                return 0
            print("仅限现场确认单颗裸舵机、无负载、接线供电正确且扭矩关闭。")
            if args.command == "set-id":
                print(f"拟将 ID {args.id} 改为 {args.new_id}；新 ID 后续须断电重连验证。")
                phrase = CONFIRM_ID
            elif args.command == "jog":
                print(f"拟以 ±{args.amplitude} 计数（约 ±{args.amplitude * 360 / 4096:.1f}°）往返点动，"
                      "结束后回原位并卸力。")
                print("点动期间舵机会真的转动：请确认输出轴悬空、周围无异物、手已离开转动范围。")
                phrase = CONFIRM_JOG
            elif args.command == "move-center":
                print("拟驱动到已有中位 2048 后卸力；不重新校零，不改 offset/mode/ID/波特率。")
                print("目标逐步推进不是固件限速；实际速度和冲击未验证。仅限正常单颗裸舵机，")
                print("输出轴悬空、手离开转动范围，外部电源可立即切断；不可用于机械异常件解卡。")
                print("失联/卡死不能保证卸力；异常立即切断舵机外部电源，不只拔 USB。")
                phrase = CONFIRM_MOVE_CENTER
            else:
                print("拟将当前位置设为 2048，不会转到已有中位；必须先确认装配参考位。")
                phrase = CONFIRM_CENTER
            # set-id 由显式 CLI 调用提供底层契约值，不查询 TTY，也不读取 stdin。
            confirm = CONFIRM_ID if args.command == "set-id" else input(f"输入「{phrase}」确认，否则取消：")
            if args.command == "set-id":
                result = change_id(bus, port=args.port, expected=before, new_id=args.new_id,
                                   confirmation=confirm, records=RECORDS)
            elif args.command == "jog":
                result = jog_test(bus, port=args.port, expected=before,
                                  amplitude=args.amplitude, confirmation=confirm,
                                  records=RECORDS)
            elif args.command == "move-center":
                result = move_center(bus, port=args.port, expected=before,
                                     confirmation=confirm, records=RECORDS)
            else:
                result = calibrate_center(bus, port=args.port, expected=before,
                                          confirmation=confirm, records=RECORDS)
            _show(result)
            return 3 if result.get("effect_observed") is False else 0
    except (ServoError, OSError, ImportError, EOFError, KeyboardInterrupt) as exc:
        print(f"\n已停止：{type(exc).__name__}: {exc}", file=sys.stderr)
        print("如写入曾开始，不要盲目重试或上力；请根据记录及现场情况先做只读排查。", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
