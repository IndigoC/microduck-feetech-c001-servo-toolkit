"""不接设备：用假 SDK 验证护栏和故障路径。"""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from dataclasses import replace
from unittest import mock

from servo_toolkit import servo
from servo_toolkit.__main__ import main


class FakeHls:
    """单颗 HLS 内存的简化模型，不模拟真实硬件、电气或掉电持久化。"""
    def __init__(self):
        self.sid = 1
        self.mem = {0: 3, 1: 46, 2: 0, 3: 10, 4: 2, 5: 1, 6: 0, 7: 0, 8: 1,
                    9: 0, 10: 0, 11: 255, 12: 15, 30: 1, 31: 0, 32: 0,
                    33: 0, 40: 0, 42: 0, 43: 0, 55: 1, 56: 0x90, 57: 9,
                    58: 0, 59: 0, 62: 75, 63: 25, 65: 0, 66: 0}
        self.calls = []
        self.fail = {}
        self.id_timeout_after_apply = False
        self.cal_timeout_after_apply = False
        self.cal_has_effect = True
        self.also_responds = set()
        self.no_motion = False  # True 时上力写目标也不更新位置，用于模拟到位超时
        self.extra_turns = False  # True 时反馈返回目标+4096，模拟全角度/多圈读数
        self.goal_enables_torque = False  # 回归模型：目标写入可能隐式开启扭矩，不代表厂商保证。

    def _fault(self, op, sid, addr):
        self.calls.append((op, sid, addr))
        key = (op, sid, addr)
        effect = self.fail.get(key)
        if isinstance(effect, list):
            effect = effect.pop(0) if effect else None
        return effect

    def ping(self, sid):
        self.calls.append(("ping", sid, None))
        if sid != self.sid:
            return 0, -6, 0
        return self.mem[3] | self.mem[4] << 8, 0, 0

    def read1ByteTxRx(self, sid, addr):
        fault = self._fault("read1", sid, addr)
        if fault is not None:
            return 0, fault, 0
        if sid not in ({self.sid} | self.also_responds):
            return 0, -6, 0
        return (sid if addr == 5 and sid in self.also_responds else self.mem.get(addr, 0)), 0, 0

    def read2ByteTxRx(self, sid, addr):
        fault = self._fault("read2", sid, addr)
        if fault is not None:
            return 0, fault, 0
        if sid != self.sid:
            return 0, -6, 0
        return self.mem.get(addr, 0) | self.mem.get(addr + 1, 0) << 8, 0, 0

    def write1ByteTxRx(self, sid, addr, value):
        fault = self._fault("write", sid, addr)
        if fault is not None:
            return fault, 0
        if sid != self.sid:
            return -6, 0
        if addr == 5:
            self.sid = self.mem[5] = value
            if self.id_timeout_after_apply:
                return -6, 0
        else:
            self.mem[addr] = value
        return 0, 0

    def write2ByteTxRx(self, sid, addr, value):
        fault = self._fault("write2", sid, addr)
        if fault is not None:
            return fault, 0
        if sid != self.sid:
            return -6, 0
        self.mem[addr] = value & 0xFF
        self.mem[addr + 1] = value >> 8 & 0xFF
        if addr == 42 and self.goal_enables_torque:
            self.mem[40] = 1
        if addr == 42 and self.mem.get(40, 0) == 1 and not self.no_motion:
            # 简化模型：上力后写目标即视为瞬间到位，不模拟速度与惯量。
            target = value + (4096 if self.extra_turns else 0)
            self.mem[56] = target & 0xFF
            self.mem[57] = target >> 8 & 0xFF
        return 0, 0

    def reOfsCal(self, sid, target):
        self.calls.append(("calibrate", sid, target))
        if sid != self.sid:
            return -6, 0
        if self.cal_has_effect:
            before = self.mem[56] | self.mem[57] << 8
            offset = before - target
            self.mem[31], self.mem[32] = offset & 255, offset >> 8 & 255
            self.mem[56], self.mem[57] = target & 255, target >> 8
        return (-6 if self.cal_timeout_after_apply else 0), 0


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeHls()
        self.bus = servo.HlsBus(self.fake, 0, -6)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.records = Path(self.tmp.name) / "records"
        self.before = self.bus.inspect(1)
        self.fake.calls.clear()
        # 仅测试环境以 mock 跳过尚未实现的设备白名单，绝不适用于实机。
        self.profile = mock.patch.object(servo, "_require_validated_profile")
        self.profile.start()
        self.addCleanup(self.profile.stop)
        # 点动默认关闭；测试环境单独放开，绝不适用于实机。
        self.armed = mock.patch.object(servo, "_require_jog_armed")
        self.armed.start()
        self.addCleanup(self.armed.stop)

    def set_id(self, **kwargs):
        options = dict(port="fake", expected=self.before, new_id=10,
                       confirmation=servo.CONFIRM_ID, records=self.records)
        options.update(kwargs)
        return servo.change_id(self.bus, **options)

    def calibrate(self, **kwargs):
        options = dict(port="fake", expected=self.before,
                       confirmation=servo.CONFIRM_CENTER, records=self.records)
        options.update(kwargs)
        return servo.calibrate_center(self.bus, **options)

    def jog(self, **kwargs):
        options = dict(port="fake", expected=self.before, amplitude=100,
                       confirmation=servo.CONFIRM_JOG, records=self.records)
        options.update(kwargs)
        return servo.jog_test(self.bus, **options)

    def use_factory_mode4(self):
        """点动只针对出厂的位置闭环模式；改模式后重取基快照。"""
        self.fake.mem[33] = 4
        self.before = self.bus.inspect(1)
        self.fake.calls.clear()

    def writes(self):
        return [call for call in self.fake.calls if call[0] in ("write", "calibrate")]

    def read_record(self):
        files = list(self.records.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        return [json.loads(line) for line in files[0].read_text().splitlines()]

    def test_inspection_is_read_only(self):
        state = self.bus.inspect(1)
        self.assertEqual((state.id, state.firmware_major, state.firmware_minor), (1, 3, 46))
        self.assertEqual((state.position, state.voltage_v, state.temperature_c), (2448, 7.5, 25))
        self.assertEqual(self.writes(), [])

    def test_non_little_endian_stops_before_multibyte_read(self):
        self.fake.mem[2] = 1
        self.fake.calls.clear()
        with self.assertRaisesRegex(servo.ServoError, "字节序"):
            self.bus.inspect(1)
        self.assertFalse(any(op == "read2" for op, _, _ in self.fake.calls))

    def test_broadcast_and_invalid_id_rejected(self):
        for value in (-1, 254, 255, True):
            with self.subTest(value=value), self.assertRaises(servo.ServoError):
                self.bus.inspect(value)
        self.assertEqual(self.writes(), [])

    def test_default_profile_blocks_all_writes(self):
        self.profile.stop()
        for action in (self.set_id, self.calibrate):
            with self.subTest(action=action), self.assertRaisesRegex(servo.ServoError, "禁止实机写入"):
                action()
        self.assertEqual(self.writes(), [])
        self.assertFalse(self.records.exists())

    def test_jog_round_trip_returns_to_base_and_releases_torque(self):
        self.use_factory_mode4()
        base = self.before.position
        result = self.jog()
        # 已卸力并回到起始位置
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(self.fake.mem[56] | self.fake.mem[57] << 8, base)
        self.assertEqual(result["drift"], 0)
        self.assertEqual(result["torque"], 0)
        self.assertIn("phase", result["checked_unchanged"])
        # 顺序：目标同步 → 上力 → +A → base → -A → base → 卸力
        ops = [c[0] for c in self.fake.calls if c[0] in ("write", "write2")]
        self.assertEqual(ops, ["write2", "write", "write2", "write2",
                               "write2", "write2", "write"])

    def test_jog_amplitude_out_of_range_writes_nothing(self):
        self.use_factory_mode4()
        for value in (0, -1, servo.JOG_MAX_AMPLITUDE + 1):
            with self.subTest(value=value), self.assertRaises(servo.ServoError):
                self.jog(amplitude=value)
        self.assertEqual([c for c in self.fake.calls if c[0] in ("write", "write2")], [])

    def test_jog_requires_factory_position_mode(self):
        # 默认假设备 mode=0，不应进入点动流程
        with self.assertRaisesRegex(servo.ServoError, "mode"):
            self.jog()
        self.assertEqual([c for c in self.fake.calls if c[0] in ("write", "write2")], [])

    def test_jog_failure_still_releases_torque(self):
        self.use_factory_mode4()
        # 第一次写目标（同步）成功，第二次（+A）通信失败
        self.fake.fail[("write2", 1, 42)] = [None, -6]
        with self.assertRaises(servo.ServoError):
            self.jog()
        self.assertEqual(self.fake.mem[40], 0)

    def test_jog_timeout_still_releases_torque(self):
        self.use_factory_mode4()
        self.fake.no_motion = True
        with mock.patch.object(servo, "JOG_TIMEOUT_S", 0.05):
            with self.assertRaisesRegex(servo.ServoError, "未到位"):
                self.jog()
        self.assertEqual(self.fake.mem[40], 0)

    def test_jog_torque_write_error_still_attempts_release(self):
        self.use_factory_mode4()
        # 上力写超时：设备可能已执行，因此必须在发送前就标记需要卸力。
        self.fake.fail[("write", 1, 40)] = [-6]
        with self.assertRaises(servo.ServoError):
            self.jog()
        releases = [c for c in self.fake.calls if c == ("write", 1, 40)]
        self.assertEqual(len(releases), 2)          # 上力一次 + 卸力一次
        self.assertEqual(self.fake.mem[40], 0)

    def test_jog_on_tainted_link_demands_power_off(self):
        self.use_factory_mode4()

        class TaintedPort:
            _microduck_tainted = True

        self.fake.portHandler = TaintedPort()
        self.fake.fail[("write2", 1, 42)] = [None, -6]
        with self.assertRaisesRegex(servo.ServoError, "切断舵机外部电源"):
            self.jog()
        # 链路已污染时不盲发卸力指令，而是要求人工断电。
        self.assertEqual(self.fake.mem[40], 1)

    def test_jog_rejects_amplitude_too_small_to_verify(self):
        # 幅度不大于判定窗时，“没动”会被当成“到位”；必须拒绝。
        self.use_factory_mode4()
        for value in (1, servo.JOG_TOLERANCE, servo.JOG_MIN_AMPLITUDE - 1):
            with self.subTest(value=value), self.assertRaises(servo.ServoError):
                self.jog(amplitude=value)
        self.assertEqual([c for c in self.fake.calls if c[0] in ("write", "write2")], [])

    def test_jog_rejects_multiturn_position_feedback(self):
        # 全角度反馈下位置可超出 0–4095；绝不能按单圈折叠后当成到位。
        self.use_factory_mode4()
        self.fake.extra_turns = True
        with self.assertRaisesRegex(servo.ServoError, "单圈"):
            self.jog()
        self.assertEqual(self.fake.mem[40], 0)

    def test_jog_refuses_to_power_on_after_position_drift(self):
        self.use_factory_mode4()
        original = servo._record_result

        def disturb(path, event, data):
            original(path, event, data)
            if event == "pre_torque_guard":
                # 模拟最后日志同步时轴被外力推走；首次目标写入也必须被挡住。
                moved = 2448 + 500
                self.fake.mem[56], self.fake.mem[57] = moved & 0xFF, moved >> 8 & 0xFF

        with mock.patch.object(servo, "_record_result", side_effect=disturb):
            with self.assertRaisesRegex(servo.ServoError, "偏离起始"):
                self.jog()
        self.assertEqual(self.fake.mem[40], 0)
        self.assertFalse(any(c[0] in ("write", "write2") for c in self.fake.calls))

    def test_jog_drift_during_pre_torque_fsync_blocks_power_on(self):
        self.profile.stop()
        self.fake.mem[4] = 31  # 使用实际型号白名单，不 mock profile。
        self.use_factory_mode4()
        original_record, original_sync = servo._record_result, servo.os.fsync
        active_event = [None]
        injected = []

        def record(path, event, data):
            active_event[0] = event
            try:
                return original_record(path, event, data)
            finally:
                active_event[0] = None

        def sync(fd):
            original_sync(fd)
            if active_event[0] == "pre_torque_guard":
                moved = self.before.position + 500
                self.fake.mem[56], self.fake.mem[57] = moved & 255, moved >> 8
                injected.append(moved)

        with mock.patch.object(servo, "_record_result", side_effect=record), \
             mock.patch.object(servo.os, "fsync", side_effect=sync):
            with self.assertRaisesRegex(servo.ServoError, "偏离起始"):
                self.jog()
        self.assertEqual(injected, [self.before.position + 500])
        self.assertFalse(any(c == ("write", 1, 40) for c in self.fake.calls))
        self.assertEqual(self.fake.mem[40], 0)

    def test_jog_final_nonzero_moving_or_speed_is_not_success(self):
        self.profile.stop()
        self.fake.mem[4] = 31
        self.use_factory_mode4()
        original_write = self.fake.write1ByteTxRx
        for register, observed in ((66, 1), (58, 25)):
            with self.subTest(register=register):
                self.fake.mem.update({66: 0, 58: 0, 59: 0})
                self.before = self.bus.inspect(1)

                def write(sid, addr, value):
                    result = original_write(sid, addr, value)
                    if addr == 40 and value == 0:
                        self.fake.mem[register] = observed
                    return result

                with mock.patch.object(self.fake, "write1ByteTxRx", side_effect=write):
                    with self.assertRaisesRegex(servo.ServoError, "尚未确认静止"):
                        self.jog()
                self.assertEqual(self.fake.mem[40], 0)

    def test_jog_gate_blocks_when_not_armed(self):
        real = self.armed.temp_original  # setUp 已 mock，这里取原函数。
        with self.assertRaisesRegex(servo.ServoError, "禁止实机上力"):
            real()

    def test_cli_jog_disabled_before_connect(self):
        # 即使终端为交互态，点动也必须在打开串口之前被拒。
        with mock.patch("servo_toolkit.__main__.connect") as connect, \
             mock.patch("sys.stdin.isatty", return_value=True), \
             mock.patch("sys.stdout.isatty", return_value=True), \
             contextlib.redirect_stderr(io.StringIO()):
            rc = main(["--port", "fake", "jog", "--id", "10"])
            self.assertEqual(rc, 2)
        connect.assert_not_called()

    def test_jog_gate_blocks_before_any_write(self):
        real = self.armed.temp_original
        self.use_factory_mode4()
        with mock.patch.object(servo, "_require_jog_armed", real):
            with self.assertRaisesRegex(servo.ServoError, "禁止实机上力"):
                self.jog()
        self.assertEqual([c for c in self.fake.calls if c[0] in ("write", "write2")], [])

    def test_change_id_success_relocks_new_id_and_records(self):
        result = self.set_id()
        self.assertEqual(self.writes(), [("write", 1, 55), ("write", 1, 5), ("write", 10, 55)])
        self.assertEqual(result["snapshot"]["id"], 10)
        self.assertEqual(result["snapshot"]["lock"], 1)
        self.assertIn("未验证", result["persistence"])
        events = self.read_record()
        self.assertEqual([row["event"] for row in events], ["before", "before_unlock", "unlock_confirmed",
                         "before_id_write", "before_relock", "relock_confirmed", "after_read", "result"])
        self.assertEqual(events[0]["snapshot"]["id"], 1)
        self.assertEqual(events[-2]["snapshot"]["id"], 10)
        self.assertEqual(events[-1]["snapshot"]["id"], 10)

    def test_wrong_confirmation_never_writes(self):
        with self.assertRaises(servo.ServoError):
            self.set_id(confirmation="yes")
        self.assertEqual(self.writes(), [])
        self.assertFalse(self.records.exists())

    def test_probe_corrupt_response_is_not_treated_as_free_id(self):
        self.fake.fail[("read1", 10, 5)] = -7
        with self.assertRaises(servo.CommunicationError):
            self.bus.probe(10)
        self.assertEqual(self.writes(), [])

    def test_id_ack_timeout_does_not_assume_which_address_to_lock(self):
        self.fake.id_timeout_after_apply = True
        with self.assertRaisesRegex(servo.ServoError, "无法确定重锁地址"):
            self.set_id()
        self.assertEqual(self.fake.sid, 10)
        self.assertEqual(self.fake.mem[55], 0)
        self.assertNotIn(("write", 10, 55), self.writes())
        self.assertEqual(self.read_record()[-1]["event"], "error")

    def test_both_ids_respond_after_write_never_blindly_relocks(self):
        self.fake.id_timeout_after_apply = True
        # 目标 ID 首次探测超时，写之后两个 ID 都响应。
        self.fake.fail[("read1", 10, 5)] = [-6]
        self.fake.also_responds.add(1)
        with self.assertRaisesRegex(servo.ServoError, "无法确定重锁地址"):
            self.set_id()
        self.assertNotIn(("write", 10, 55), self.writes())
        self.assertEqual(self.fake.mem[55], 0)
        self.assertEqual(self.read_record()[-1]["event"], "error")

    def test_unlock_timeout_relocks_old_and_never_writes_id(self):
        self.fake.fail[("write", 1, 55)] = [-6, None]
        with self.assertRaisesRegex(servo.ServoError, "结果不确定"):
            self.set_id()
        self.assertNotIn(("write", 1, 5), self.writes())
        self.assertEqual(self.fake.mem[55], 1)

    def test_keyboard_interrupt_during_before_relock_still_attempts_relock(self):
        real_record = servo._record_result
        def interrupted(path, event, data):
            if event == "before_relock":
                raise KeyboardInterrupt()
            return real_record(path, event, data)
        with mock.patch.object(servo, "_record_result", side_effect=interrupted):
            with self.assertRaises(servo.ServoError):
                self.calibrate()
        self.assertEqual(self.fake.mem[55], 1)
        self.assertEqual(sum(c == ("write", 1, 55) for c in self.fake.calls), 2)

    def test_unlock_readback_failure_relocks_and_stops(self):
        # inspect 首次读锁通过，解锁后的回读超时，重锁回读恢复。
        self.fake.fail[("read1", 1, 55)] = [None, -6, None]
        with self.assertRaises(servo.ServoError):
            self.set_id()
        self.assertNotIn(("write", 1, 5), self.writes())
        self.assertEqual(self.fake.mem[55], 1)

    def test_relock_failure_is_not_success(self):
        self.fake.fail[("write", 10, 55)] = -6
        with self.assertRaisesRegex(servo.ServoError, "重锁失败"):
            self.set_id()
        self.assertEqual(self.read_record()[-1]["event"], "error")

    def test_cannot_record_means_no_unlock_or_write(self):
        with mock.patch.object(servo, "_record_start", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.set_id()
        self.assertEqual(self.writes(), [])

    def test_directory_fsync_failure_stops_before_writing(self):
        with mock.patch.object(servo, "_sync_dir", side_effect=OSError("dir sync")):
            with self.assertRaisesRegex(OSError, "dir sync"):
                self.set_id()
        self.assertEqual(self.writes(), [])

    def test_before_unlock_log_failure_never_sends_lock_write(self):
        real_record = servo._record_result
        def fail_stage(path, event, data):
            if event == "before_unlock":
                raise OSError("stage log")
            return real_record(path, event, data)
        with mock.patch.object(servo, "_record_result", side_effect=fail_stage):
            with self.assertRaisesRegex(servo.ServoError, "未开始写入"):
                self.set_id()
        self.assertEqual(self.writes(), [])

    def test_relock_ack_timeout_readback_does_not_claim_id_success(self):
        original = self.fake.write1ByteTxRx
        def write(sid, addr, value):
            result, status = original(sid, addr, value)
            if sid == 10 and addr == 55 and value == 1:
                return -6, 0  # 假设备已写锁，仅 ACK 丢失。
            return result, status
        self.fake.write1ByteTxRx = write
        result = self.set_id()
        self.assertEqual(self.fake.mem[55], 1)
        self.assertEqual(result["new_id"], 10)
        self.assertIn("未验证", result["persistence"])
        self.assertEqual(sum(c == ("write", 1, 5) for c in self.fake.calls), 1)

    def test_calibration_passes_when_position_and_offset_change(self):
        result = self.calibrate()
        self.assertEqual(result["after"]["position"], 2048)
        self.assertNotEqual(result["before"]["offset_raw"], result["after"]["offset_raw"])
        self.assertEqual(self.writes(), [("write", 1, 55), ("calibrate", 1, 2048), ("write", 1, 55)])
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(self.read_record()[-1]["event"], "result")
        self.assertFalse(result["effect_observed"])
        self.assertIn("不能判定", result["state"])

    def test_calibration_ack_success_but_no_effect_is_failure(self):
        self.fake.cal_has_effect = False
        with self.assertRaisesRegex(servo.ServoError, "位置回读"):
            self.calibrate()
        self.assertEqual(self.fake.mem[55], 1)
        events = self.read_record()
        self.assertEqual([row["event"] for row in events], ["before", "before_unlock", "unlock_confirmed",
                         "before_calibrate", "before_relock", "relock_confirmed", "after_read", "error"])
        self.assertEqual(events[-2]["snapshot"]["position"], 2448)

    def test_already_at_center_cannot_prove_command_effect(self):
        self.fake.mem[56], self.fake.mem[57] = 0, 8
        self.before = self.bus.inspect(1)
        self.fake.cal_has_effect = False
        result = self.calibrate()
        self.assertIn("无法证明", result["state"])
        self.assertFalse(result["effect_observed"])

    def test_invalid_after_offset_is_recorded_and_rejected(self):
        def invalid_calibration(sid, target):
            self.fake.calls.append(("calibrate", sid, target))
            self.fake.mem[56], self.fake.mem[57] = 0, 8
            self.fake.mem[31], self.fake.mem[32] = 0xff, 0x7f
            return 0, 0
        self.fake.reOfsCal = invalid_calibration
        with self.assertRaisesRegex(servo.ServoError, "含保留位"):
            self.calibrate()
        events = self.read_record()
        self.assertEqual([row["event"] for row in events], ["before", "before_unlock", "unlock_confirmed",
                         "before_calibrate", "before_relock", "relock_confirmed", "after_read", "error"])
        self.assertEqual(events[-2]["snapshot"]["offset_raw"], 0x7fff)

    def test_calibration_timeout_after_apply_relocks_but_is_not_success(self):
        self.fake.cal_timeout_after_apply = True
        with self.assertRaisesRegex(servo.ServoError, "结果不确定"):
            self.calibrate()
        self.assertEqual(self.fake.mem[55], 1)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(self.read_record()[-1]["event"], "error")

    def test_reject_preconditions_without_writing(self):
        # 同一字节地址的各状态分别试；使用新的确认快照，避免只测到状态变化。
        for addr, value in [(40, 1), (40, 2), (55, 0), (8, 0), (2, 1),
                            (6, 3), (62, 39), (65, 1), (66, 1),
                            (1, 42), (33, 1), (33, 4), (18, 0x10), (30, 2),
                            (32, 0x70), (12, 0x04)]:
            with self.subTest(addr=addr, value=value):
                fake = FakeHls()
                fake.mem[addr] = value
                bus = servo.HlsBus(fake, 0, -6)
                if addr == 2:
                    with self.assertRaisesRegex(servo.ServoError, "字节序"):
                        bus.inspect(1)
                    continue
                snap = bus.inspect(1)
                fake.calls.clear()
                with self.assertRaises(servo.ServoError):
                    servo.calibrate_center(bus, port="fake", expected=snap,
                        confirmation=servo.CONFIRM_CENTER, records=self.records)
                self.assertFalse([c for c in fake.calls if c[0] in ("write", "calibrate")])

    def test_reference_position_changed_after_confirmation_rejected(self):
        self.fake.mem[56] += 5
        with self.assertRaisesRegex(servo.ServoError, "位置发生变化"):
            self.calibrate()
        self.assertEqual(self.writes(), [])

    def test_torque_change_immediately_before_calibration_stops(self):
        # inspect 快照、解锁前 inspect、随后二次读取扭矩返回异常。
        self.fake.fail[("read1", 1, 40)] = [None, -6]
        with self.assertRaises(servo.ServoError):
            self.calibrate()
        self.assertFalse(any(c[0] == "calibrate" for c in self.fake.calls))
        self.assertEqual(self.fake.mem[55], 1)

    def test_cli_write_gates_still_run_before_connect(self):
        # 非 tty、以及未实现的 set-baud，必须在打开串口前退出。
        with mock.patch("servo_toolkit.__main__.connect") as connect, \
             mock.patch("sys.stdin.isatty", return_value=False), \
             contextlib.redirect_stderr(io.StringIO()):
            for command in (["calibrate-center", "--id", "1"], ["set-baud", "--id", "1"]):
                rc = main(["--port", "fake", *command])
                self.assertEqual(rc, 2)
        connect.assert_not_called()

    def test_verified_profile_accepts_and_unlisted_rejects(self):
        # 实机读回的那组型号读数应放行；其它读数一律拒绝。
        real = self.profile.temp_original  # setUp 已把该门禁 mock 掉，这里取原函数。
        snap = self.bus.inspect(1)
        real(replace(snap, ping_model_number=7946, model_low=10, model_high=31))
        with self.assertRaisesRegex(servo.ServoError, "白名单"):
            real(snap)


class MoveCenterTests(unittest.TestCase):
    """仅假设备；使用真实 profile 校验，不继承旧测试的 profile/jog mock。"""
    def setUp(self):
        self.fake = FakeHls()
        self.fake.mem.update({4: 31, 7: 253, 18: 52, 33: 4, 56: 24, 57: 0})
        self.bus = servo.HlsBus(self.fake, 0, -6)
        self.before = self.bus.inspect(1)
        self.fake.calls.clear()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.records = Path(self.tmp.name) / "records"
        self.gate = mock.patch.object(servo, "MOVE_CENTER_ARMED", True)
        self.gate.start()
        self.addCleanup(self.gate.stop)
        self.now = 0.0
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(servo.time, "monotonic", side_effect=lambda: self.now).start()
        mock.patch.object(servo.time, "sleep", side_effect=self.sleep).start()
        self.targets = []
        original = self.fake.write2ByteTxRx

        def write(sid, addr, value):
            pos = servo.signed_magnitude(self.fake.mem[56] | self.fake.mem[57] << 8)
            self.targets.append((value, pos, self.fake.mem[40], self.now))
            return original(sid, addr, value)
        mock.patch.object(self.fake, "write2ByteTxRx", side_effect=write).start()

    def sleep(self, seconds):
        self.now += max(0.001, seconds)

    def prepare(self, pos):
        self.fake.mem[56], self.fake.mem[57] = pos & 255, pos >> 8
        self.before = self.bus.inspect(1)
        self.fake.calls.clear()

    def move(self, **kwargs):
        options = dict(port="fake", expected=self.before,
                       confirmation=servo.CONFIRM_MOVE_CENTER, records=self.records)
        options.update(kwargs)
        return servo.move_center(self.bus, **options)

    def writes(self):
        return [c for c in self.fake.calls if c[0] in ("write", "write2", "calibrate")]

    def rows(self):
        return [json.loads(s) for p in self.records.glob("*.jsonl")
                for s in p.read_text(encoding="utf-8").splitlines()]

    def test_center_both_directions_and_single_turn_endpoints(self):
        for start in (0, 24, 4095):
            with self.subTest(start=start):
                self.prepare(start)
                self.targets.clear()
                result = self.move()
                self.assertEqual(result["final_position"], 2048)
                self.assertEqual(result["torque"], 0)
                self.assertTrue(all(c[2] in (40, 42) for c in self.writes()))
                self.assertEqual(len(result["checked_unchanged"]), 17)
                motion = [t for t in self.targets if t[2] == 1]
                self.assertEqual(motion[-1][0], 2048)
                for target, position, _, _ in motion:
                    self.assertLessEqual(abs(target - position), servo.MOVE_CENTER_STEP)
                    self.assertGreaterEqual((target - position) * (2048 - position), 0)
                for a, b in zip(motion, motion[1:]):
                    self.assertGreaterEqual(b[3] - a[3] + 1e-8, servo.MOVE_CENTER_INTERVAL_S)

    def test_requested_one_second_wait_and_bounded_total_timeout(self):
        self.assertEqual(servo.MOVE_CENTER_INTERVAL_S, 1.0)
        self.assertEqual(servo.MOVE_CENTER_TOTAL_TIMEOUT_S, 180.0)
        self.assertEqual(servo.MOVE_CENTER_STEP_TIMEOUT_S, 1.0)
        self.assertEqual(servo.MOVE_CENTER_STEP, 32)
        self.assertEqual(servo.MOVE_CENTER_TOLERANCE, 3)

    def test_final_step_waits_one_second_after_two_settled_samples(self):
        self.prepare(2040)
        original = self.fake.read2ByteTxRx
        samples = []

        def read(sid, addr):
            result = original(sid, addr)
            if addr == 58 and len(self.targets) == 2 and self.fake.mem[40] == 1:
                samples.append(self.now)
            return result

        with mock.patch.object(self.fake, "read2ByteTxRx", side_effect=read):
            self.assertEqual(self.move()["final_position"], 2048)
        self.assertEqual(len(samples), 3)  # 两次到位采样，再一次等待后的步间复核。
        self.assertGreaterEqual(samples[2] - samples[1] + 1e-9, 1.0)
        self.assertEqual(self.fake.mem[40], 0)

    def test_slow_endpoint_moves_with_allowed_lag_fit_revised_total_timeout(self):
        original = self.fake.write2ByteTxRx.side_effect
        for start in (0, 4095):
            with self.subTest(start=start):
                self.prepare(start)
                self.targets.clear()
                self.now = 0.0

                def lag(sid, addr, target):
                    previous = self.fake.mem[56] | self.fake.mem[57] << 8
                    result = original(sid, addr, target)
                    if self.fake.mem[40] == 1:
                        self.now += 0.9  # 发送/ACK 耗时仍计入单步 1 s。
                        pos = target - 3 if target > previous else target + 3
                        self.fake.mem[56], self.fake.mem[57] = pos & 255, pos >> 8
                    return result

                with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=lag):
                    result = self.move()
                self.assertEqual(result["target"], 2048)
                self.assertLessEqual(abs(result["final_position"] - 2048), 3)
                self.assertGreater(self.now, 130.0)
                self.assertLess(self.now, 180.0)
                self.assertEqual(result["steps"], 71)
                self.assertEqual(self.fake.mem[40], 0)

    def test_interstep_checks_wait_full_interval_after_settled_feedback(self):
        self.prepare(2000)
        original = self.fake.read2ByteTxRx
        samples, settled_at, checks = [], [], []

        def read(sid, addr):
            result = original(sid, addr)
            if len(self.targets) == 2:  # 同步目标之后的第一个运动目标。
                if addr == 58:
                    if not samples:
                        self.now += 0.15  # 运动/反馈耗时超过原来的发令间隔。
                    samples.append(self.now)
                    if len(samples) == 2:
                        settled_at.append(self.now)
                elif addr == 42 and settled_at:
                    checks.append(self.now)
            return result

        with mock.patch.object(self.fake, "read2ByteTxRx", side_effect=read):
            self.assertEqual(self.move()["final_position"], 2048)
        self.assertEqual(len(settled_at), 1)
        self.assertTrue(checks)
        self.assertGreaterEqual(checks[0] - settled_at[0] + 1e-9,
                                servo.MOVE_CENTER_INTERVAL_S)
        motion = [t for t in self.targets if t[2] == 1]
        self.assertEqual([t[0] for t in motion], [2032, 2048])
        self.assertGreaterEqual(motion[1][3] - settled_at[0] + 1e-9,
                                servo.MOVE_CENTER_INTERVAL_S)

    def test_nonzero_interstep_feedback_after_wait_still_stops(self):
        for address in (66, 58):
            with self.subTest(address=address):
                self.prepare(2000)
                self.targets.clear()

                def sleep(seconds):
                    self.sleep(seconds)
                    if seconds > 0.04 and self.fake.mem[40] == 1:
                        self.fake.mem[address] = 1

                try:
                    with mock.patch.object(servo.time, "sleep", side_effect=sleep), \
                         self.assertRaisesRegex(servo.ServoError, "步间尚未静止"):
                        self.move()
                    self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
                    self.assertEqual(self.fake.mem[40], 0)
                    self.assertEqual(self.rows()[-1]["event"], "error")
                finally:
                    self.fake.mem[address] = 0

    def test_post_settle_wait_counts_toward_total_deadline(self):
        self.prepare(2000)
        with mock.patch.object(servo, "MOVE_CENTER_TOTAL_TIMEOUT_S", 0.11), \
             self.assertRaisesRegex(servo.ServoError, "总超时"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(len(self.rows()[-2]["steps"]), 1)

    def test_already_centered_reads_only(self):
        for pos in (2045, 2048, 2051):
            self.prepare(pos)
            result = self.move()
            self.assertEqual(result["final_position"], pos)
            self.assertEqual(self.writes(), [])
            self.assertFalse(self.records.exists())

    def test_independent_module_gate_blocks_all_bus_access(self):
        with mock.patch.object(servo, "MOVE_CENTER_ARMED", False), \
             mock.patch.object(servo, "JOG_ARMED", True), \
             self.assertRaisesRegex(servo.ServoError, "禁止实机上力"):
            self.move()
        self.assertEqual(self.fake.calls, [])
        self.assertFalse(self.records.exists())

    def test_bad_confirmation_writes_nothing(self):
        with self.assertRaisesRegex(servo.ServoError, "确认语"):
            self.move(confirmation="yes")
        self.assertEqual(self.fake.calls, [])

    def test_cli_gate_and_non_tty_reject_before_connect(self):
        import servo_toolkit.__main__ as cli
        with mock.patch.object(cli, "connect") as connect, \
             mock.patch.object(cli, "JOG_ENABLED", True), \
             contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(["--port", "fake", "move-center", "--id", "12"]), 2)
            with mock.patch.object(cli, "MOVE_CENTER_ENABLED", True), \
                 mock.patch("sys.stdin.isatty", return_value=False):
                self.assertEqual(cli.main(["--port", "fake", "move-center", "--id", "12"]), 2)
        connect.assert_not_called()

    def test_cli_routes_move_not_calibration(self):
        import servo_toolkit.__main__ as cli
        with mock.patch.object(cli, "MOVE_CENTER_ENABLED", True), \
             mock.patch.object(cli, "connect") as connect, \
             mock.patch.object(cli, "move_center", return_value={}) as move, \
             mock.patch.object(cli, "calibrate_center") as calibrate, \
             contextlib.redirect_stdout(io.StringIO()), \
             mock.patch("sys.stdin.isatty", return_value=True), \
             mock.patch("sys.stdout.isatty", return_value=True), \
             mock.patch("builtins.input", return_value=servo.CONFIRM_MOVE_CENTER):
            connect.return_value.__enter__.return_value.inspect.return_value = self.before
            self.assertEqual(cli.main(["--port", "fake", "move-center", "--id", "1"]), 0)
        move.assert_called_once()
        calibrate.assert_not_called()

    def test_profile_config_motion_and_limits_reject_before_writes(self):
        for addr, value in ((4, 2), (1, 45), (33, 0), (18, 0), (30, 2),
                            (6, 3), (8, 0), (55, 0), (40, 1), (58, 1),
                            (66, 1), (65, 1), (62, 39), (12, 4), (57, 16),
                            (9, 100)):
            old = self.fake.mem.get(addr, 0)
            with self.subTest(addr=addr):
                self.fake.mem[addr] = value
                self.before = self.bus.inspect(1)
                self.fake.calls.clear()
                with self.assertRaises(servo.ServoError):
                    self.move()
                self.assertEqual(self.writes(), [])
            self.fake.mem[addr] = old

    def test_confirmation_drift_is_not_silently_accepted(self):
        self.fake.mem[56] += 10
        with self.assertRaisesRegex(servo.ServoError, "位置变化"):
            self.move()
        self.assertEqual(self.writes(), [])

    def test_disk_failure_before_motion_writes_nothing(self):
        with mock.patch.object(servo, "_record_start", side_effect=OSError("disk")), \
             self.assertRaises(OSError):
            self.move()
        self.assertEqual(self.writes(), [])

    def test_fsync_drift_is_guarded_before_torque(self):
        real_record, real_sync = servo._record_result, servo.os.fsync
        active = [False]

        def record(path, event, data):
            active[0] = event == "pre_torque_guard"
            try:
                return real_record(path, event, data)
            finally:
                active[0] = False

        def sync(fd):
            real_sync(fd)
            if active[0]:
                self.fake.mem[56] += 50
        with mock.patch.object(servo, "_record_result", side_effect=record), \
             mock.patch.object(servo.os, "fsync", side_effect=sync), \
             self.assertRaisesRegex(servo.ServoError, "上力前目标/位置变化"):
            self.move()
        self.assertFalse(any(c == ("write", 1, 40) for c in self.fake.calls))

    def test_no_disk_sync_while_powered(self):
        real_sync = servo.os.fsync

        def sync(fd):
            self.assertEqual(self.fake.mem[40], 0, "上力期间不应有 fsync")
            return real_sync(fd)
        with mock.patch.object(servo.os, "fsync", side_effect=sync):
            self.move()
        self.assertEqual([r["event"] for r in self.rows()],
                         ["before", "pre_torque_guard", "after_read", "result"])

    def test_no_motion_times_out_without_accumulating_targets(self):
        self.fake.no_motion = True
        with self.assertRaisesRegex(servo.ServoError, "未到位"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(self.rows()[-1]["event"], "error")

    def test_small_last_step_still_cannot_accept_no_motion_outside_window(self):
        self.prepare(2044)
        self.fake.no_motion = True
        with self.assertRaisesRegex(servo.ServoError, "未到位"):
            self.move()
        self.assertEqual(self.fake.mem[40], 0)

    def test_lagged_feedback_prevents_next_target(self):
        original = self.fake.write2ByteTxRx.side_effect

        def lag(sid, addr, target):
            result = original(sid, addr, target)
            if self.fake.mem[40] == 1:
                pos = target - 8
                self.fake.mem[56], self.fake.mem[57] = pos & 255, pos >> 8
            return result
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=lag), \
             self.assertRaisesRegex(servo.ServoError, "未到位"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
        self.assertEqual(self.fake.mem[40], 0)

    def test_tolerated_lag_uses_actual_position_not_old_target(self):
        original = self.fake.write2ByteTxRx.side_effect

        def lag(sid, addr, target):
            result = original(sid, addr, target)
            if self.fake.mem[40] == 1:
                pos = target - 2
                self.fake.mem[56], self.fake.mem[57] = pos & 255, pos >> 8
            return result
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=lag):
            result = self.move()
        self.assertEqual(result["final_position"], 2046)
        self.assertTrue(all(abs(t - p) <= 32 for t, p, on, _ in self.targets if on))
        self.assertEqual(self.targets[-1][0], 2048)

    def test_reverse_and_multiturn_feedback_release_and_stop(self):
        original = self.fake.write2ByteTxRx.side_effect
        for pos in (0, 4120):
            self.prepare(24)
            self.targets.clear()

            def reverse(sid, addr, target):
                result = original(sid, addr, target)
                if self.fake.mem[40] == 1:
                    self.fake.mem[56], self.fake.mem[57] = pos & 255, pos >> 8
                return result
            with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=reverse), \
                 self.assertRaises(servo.ServoError):
                self.move()
            self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
            self.assertEqual(self.fake.mem[40], 0)

    def test_stuck_midway_stops_no_auto_return(self):
        original = self.fake.write2ByteTxRx.side_effect

        def stuck(sid, addr, target):
            if len([t for t in self.targets if t[2] == 1]) >= 2:
                self.fake.no_motion = True
            return original(sid, addr, target)
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=stuck), \
             self.assertRaisesRegex(servo.ServoError, "未到位"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 3)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(len(self.rows()[-2]["steps"]), 2)

    def test_total_deadline_covers_wait_and_no_next_goal(self):
        with mock.patch.object(servo, "MOVE_CENTER_TOTAL_TIMEOUT_S", 0.08), \
             self.assertRaisesRegex(servo.ServoError, "总超时"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
        self.assertEqual(self.fake.mem[40], 0)

    def test_total_deadline_covers_serial_reads_before_first_goal(self):
        original = self.fake.read2ByteTxRx

        def slow(sid, addr):
            if self.fake.mem[40] == 1:
                self.now += 0.2
            return original(sid, addr)
        with mock.patch.object(self.fake, "read2ByteTxRx", side_effect=slow), \
             mock.patch.object(servo, "MOVE_CENTER_TOTAL_TIMEOUT_S", 0.1), \
             self.assertRaisesRegex(servo.ServoError, "总超时"):
            self.move()
        self.assertEqual([t for t in self.targets if t[2] == 1], [])
        self.assertEqual(self.fake.mem[40], 0)

    def test_write_ack_time_counts_toward_total_deadline(self):
        original = self.fake.write2ByteTxRx.side_effect

        def slow(sid, addr, target):
            result = original(sid, addr, target)
            if self.fake.mem[40] == 1:
                self.now += 0.2
            return result
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=slow), \
             mock.patch.object(servo, "MOVE_CENTER_TOTAL_TIMEOUT_S", 0.1), \
             self.assertRaisesRegex(servo.ServoError, "总超时"):
            self.move()
        self.assertEqual(len([t for t in self.targets if t[2] == 1]), 1)
        self.assertEqual(self.fake.mem[40], 0)

    def test_byte_level_move_uses_only_atomic_goal_and_torque_frames(self):
        self.prepare(2040)
        port = MotionMemoryPort(self.fake)
        from servo_toolkit.strict_sdk import StrictHls
        self.bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, -6)
        self.before = self.bus.inspect(1)
        result = self.move()
        self.assertEqual(result["final_position"], 2048)
        writes = [p for p in port.packets if p[4] == 3]
        self.assertTrue(all(p[2] == 1 for p in port.packets))
        self.assertTrue(all(p[4] in (1, 2, 3) for p in port.packets))
        self.assertEqual([p[5] for p in writes], [42, 40, 42, 40])
        self.assertEqual([len(p) for p in writes], [9, 8, 9, 8])
        self.assertEqual(writes[-2][6] | writes[-2][7] << 8, 2048)
        self.assertEqual(self.fake.mem[40], 0)

    def test_delayed_wire_delivery_does_not_consume_next_target_wait(self):
        self.prepare(2000)

        class DelayedPort(MotionMemoryPort):
            def __init__(port, device):
                super().__init__(device)
                port.deliveries = []

            def writePort(port, data):
                powered_goal = data[4] == 3 and data[5] == 42 and port.device.mem[40] == 1
                if powered_goal:
                    if not port.deliveries:
                        self.sleep(0.2)  # 写入前阻塞，尚未交付目标；不是 ACK 后延迟。
                    port.deliveries.append((data[6] | data[7] << 8, self.now))
                return super().writePort(data)

        port = DelayedPort(self.fake)
        from servo_toolkit.strict_sdk import StrictHls
        self.bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, -6)
        self.before = self.bus.inspect(1)
        self.assertEqual(self.move()["final_position"], 2048)
        self.assertEqual([target for target, _ in port.deliveries], [2032, 2048])
        self.assertGreaterEqual(port.deliveries[1][1] - port.deliveries[0][1] + 1e-9,
                                servo.MOVE_CENTER_INTERVAL_S)

    def test_byte_level_lost_motion_ack_taints_without_cleanup_packet(self):
        port = MotionMemoryPort(self.fake, drop_motion_ack=True)
        from servo_toolkit.strict_sdk import StrictHls
        self.bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, -6)
        self.before = self.bus.inspect(1)
        with self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
            self.move()
        self.assertTrue(port._microduck_tainted)
        self.assertEqual(port.packets[-1][4:6], bytes([3, 42]))
        self.assertEqual(self.fake.mem[40], 1)
        self.assertEqual([r["event"] for r in self.rows()], ["before", "pre_torque_guard"])

    def test_taint_on_torque_ack_never_sends_more_packets_or_logs(self):
        class Port:
            _microduck_tainted = False
        self.fake.portHandler = Port()
        original = self.fake.write1ByteTxRx
        calls_at_taint = []

        def taint(sid, addr, value):
            result = original(sid, addr, value)
            if addr == 40 and value == 1:
                self.fake.portHandler._microduck_tainted = True
                calls_at_taint[:] = self.fake.calls
                return -6, 0
            return result
        with mock.patch.object(self.fake, "write1ByteTxRx", side_effect=taint), \
             self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
            self.move()
        self.assertEqual(self.fake.calls, calls_at_taint)
        self.assertEqual(self.fake.mem[40], 1)  # 不可因假设备状态推断已安全卸力。
        self.assertEqual([r["event"] for r in self.rows()], ["before", "pre_torque_guard"])

    def test_release_failure_warns_without_any_post_failure_log(self):
        self.fake.fail[("write", 1, 40)] = [None, -6]
        with self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
            self.move()
        self.assertEqual([r["event"] for r in self.rows()], ["before", "pre_torque_guard"])
        self.assertEqual(self.fake.mem[40], 1)

    def test_keyboard_interrupt_attempts_release_no_retry(self):
        original = self.fake.write2ByteTxRx.side_effect

        def interrupt(sid, addr, value):
            if self.fake.mem[40] == 1:
                raise KeyboardInterrupt()
            return original(sid, addr, value)
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=interrupt), \
             self.assertRaises(servo.ServoError):
            self.move()
        self.assertEqual(self.fake.mem[40], 0)

    def test_final_torque_reappearing_warns_without_post_read_log(self):
        original_inspect, original_write = self.bus.inspect, self.fake.write1ByteTxRx
        released = [False]

        def write(sid, addr, value):
            result = original_write(sid, addr, value)
            if addr == 40 and value == 0:
                released[0] = True
            return result

        def inspect(sid):
            snapshot = original_inspect(sid)
            return replace(snapshot, torque=1) if released[0] else snapshot
        with mock.patch.object(self.fake, "write1ByteTxRx", side_effect=write), \
             mock.patch.object(self.bus, "inspect", side_effect=inspect), \
             self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
            self.move()
        self.assertEqual([r["event"] for r in self.rows()], ["before", "pre_torque_guard"])

    def test_unpowered_final_drift_motion_alarm_and_config_changes_fail(self):
        self.prepare(2040)
        original = self.fake.write1ByteTxRx
        for addr, value in ((56, 64), (58, 1), (66, 1), (65, 1), (31, 1), (6, 3)):
            with self.subTest(addr=addr):
                self.fake.mem.update({40: 0, 31: 0, 6: 0, 58: 0, 66: 0, 65: 0})
                self.prepare(2040)

                def alter(sid, address, written):
                    result = original(sid, address, written)
                    if address == 40 and written == 0:
                        self.fake.mem[addr] = value
                    return result
                with mock.patch.object(self.fake, "write1ByteTxRx", side_effect=alter), \
                     self.assertRaises(servo.ServoError):
                    self.move()
                self.assertEqual(self.fake.mem[40], 0)
        self.assertFalse(any(r["event"] == "result" for r in self.rows()))


class ImplicitGoalPowerTests(unittest.TestCase):
    """目标写入即可能上力的防御模型；不证明真实固件因果或运动安全。"""
    def setUp(self):
        self.fake = FakeHls()
        self.fake.mem.update({4: 31, 7: 253, 18: 52, 33: 4, 56: 119, 57: 0})
        self.fake.goal_enables_torque = True
        self.bus = servo.HlsBus(self.fake, 0, -6)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.records = Path(self.tmp.name) / "records"
        self.now = 0.0
        self.events = []
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(servo, "MOVE_CENTER_ARMED", True).start()
        mock.patch.object(servo, "JOG_ARMED", True).start()
        mock.patch.object(servo.time, "monotonic", side_effect=lambda: self.now).start()
        mock.patch.object(servo.time, "sleep", side_effect=self.sleep).start()

    def sleep(self, seconds):
        self.now += max(.001, seconds)

    def run_motion(self, action):
        expected = self.bus.inspect(1)
        if action == "move-center":
            return servo.move_center(self.bus, port="fake", expected=expected,
                                     confirmation=servo.CONFIRM_MOVE_CENTER, records=self.records)
        return servo.jog_test(self.bus, port="fake", expected=expected, amplitude=50,
                              confirmation=servo.CONFIRM_JOG, records=self.records)

    def byte_bus(self, port=None):
        from servo_toolkit.strict_sdk import StrictHls
        port = port or MotionMemoryPort(self.fake)
        self.bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, -6)
        return port

    def test_implicit_enable_can_center_and_jog_then_release(self):
        for action in ("move-center", "jog"):
            with self.subTest(action=action):
                self.fake.mem.update({40: 0, 56: 119, 57: 0})
                port = self.byte_bus()
                result = self.run_motion(action)
                self.assertEqual(result["torque"], 0)
                self.assertEqual(self.fake.mem[40], 0)
                self.assertEqual(result["final_position"], 2048 if action == "move-center" else 119)
                writes = [p for p in port.packets if p[4] == 3]
                self.assertEqual(writes[0][5:8], bytes([42, 119, 0]))
                self.assertTrue(all(p[5] in (40, 42) for p in writes))
                self.assertTrue(all(len(p) == (9 if p[5] == 42 else 8) for p in writes))
                # 已自动上力时不再补发显式开扭矩；最终仅写一次关闭。
                self.assertEqual([p[6] for p in writes if p[5] == 40], [0])
                self.assertEqual((self.fake.mem[31], self.fake.mem[33], self.fake.mem[18]), (0, 4, 52))

    def test_no_record_or_fsync_after_first_goal_until_release(self):
        original_record, original_sync = servo._record_result, servo.os.fsync
        def record(*args, **kwargs):
            self.assertEqual(self.fake.mem[40], 0, "目标写入后的带力阶段不得落盘")
            return original_record(*args, **kwargs)
        def sync(*args, **kwargs):
            self.assertEqual(self.fake.mem[40], 0, "目标写入后的带力阶段不得 fsync")
            return original_sync(*args, **kwargs)
        with mock.patch.object(servo, "_record_result", side_effect=record), \
             mock.patch.object(servo.os, "fsync", side_effect=sync):
            self.run_motion("move-center")
            self.run_motion("jog")

    def test_pre_goal_guard_rejects_drift_before_any_write(self):
        original = servo._record_result
        def disturb(path, event, data):
            original(path, event, data)
            if event == "pre_torque_guard":
                self.fake.mem[56] += 20
        port = self.byte_bus()
        with mock.patch.object(servo, "_record_result", side_effect=disturb), \
             self.assertRaises(servo.ServoError):
            self.run_motion("move-center")
        self.assertEqual([p for p in port.packets if p[4] == 3], [])

    def test_initial_torque_one_is_still_rejected_without_writes(self):
        self.fake.mem[40] = 1
        port = self.byte_bus()
        for action in ("move-center", "jog"):
            with self.subTest(action=action), self.assertRaises(servo.ServoError):
                self.run_motion(action)
        self.assertEqual([p for p in port.packets if p[4] == 3], [])

    def test_post_goal_guard_failure_releases_without_motion_targets(self):
        original = self.fake.write2ByteTxRx
        for action in ("move-center", "jog"):
            with self.subTest(action=action):
                self.fake.mem.update({40: 0, 56: 119, 57: 0, 65: 0})
                port = self.byte_bus()
                def alarm(sid, addr, value):
                    result = original(sid, addr, value)
                    self.fake.mem[65] = 1
                    return result
                with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=alarm), \
                     self.assertRaises(servo.ServoError):
                    self.run_motion(action)
                writes = [p for p in port.packets if p[4] == 3]
                self.assertEqual([p[5] for p in writes], [42, 40])
                self.assertEqual(writes[-1][6], 0)
                self.assertEqual(self.fake.mem[40], 0)
                rows = [json.loads(line) for p in self.records.glob("*.jsonl")
                        for line in p.read_text().splitlines()]
                self.assertFalse(any("未发送上力指令" in r.get("error", "") for r in rows))

    def test_implicit_enable_lost_first_goal_ack_warns_without_further_io(self):
        class LostFirstGoal(MotionMemoryPort):
            def writePort(port, data):
                result = super().writePort(data)
                if data[4] == 3 and data[5] == 42:
                    port.reply.clear()
                return result
        for action in ("move-center", "jog"):
            with self.subTest(action=action):
                self.fake.mem.update({40: 0, 56: 119, 57: 0})
                port = self.byte_bus(LostFirstGoal(self.fake))
                with self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
                    self.run_motion(action)
                self.assertEqual(port.packets[-1][4:6], bytes([3, 42]))
                self.assertTrue(port._microduck_tainted)
                self.assertEqual(self.fake.mem[40], 1)
                rows = [json.loads(line) for p in self.records.glob("*.jsonl")
                        for line in p.read_text().splitlines()]
                self.assertEqual(set(r["event"] for r in rows), {"before", "pre_torque_guard"})

    def test_implicit_enable_first_goal_exception_still_releases_on_trusted_link(self):
        original = self.fake.write2ByteTxRx
        def interrupted(sid, addr, value):
            original(sid, addr, value)
            raise KeyboardInterrupt()
        for action in ("move-center", "jog"):
            with self.subTest(action=action):
                self.fake.mem.update({40: 0, 56: 119, 57: 0})
                self.bus = servo.HlsBus(self.fake, 0, -6)
                with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=interrupted), \
                     self.assertRaises(servo.ServoError):
                    self.run_motion(action)
                self.assertEqual(self.fake.mem[40], 0)

    def test_implicit_enable_failed_release_never_logs_after_first_write(self):
        original_record = servo._record_result
        def record(*args, **kwargs):
            self.assertEqual(self.fake.mem[40], 0, "失去卸力确认时不得落盘")
            return original_record(*args, **kwargs)
        for action in ("move-center", "jog"):
            with self.subTest(action=action):
                self.fake.mem.update({40: 0, 56: 119, 57: 0})
                self.fake.fail[("write", 1, 40)] = [-6]
                self.bus = servo.HlsBus(self.fake, 0, -6)
                with mock.patch.object(servo, "_record_result", side_effect=record), \
                     self.assertRaisesRegex(servo.ServoError, "立即切断舵机外部电源"):
                    self.run_motion(action)
                self.assertEqual(self.fake.mem[40], 1)

    def test_post_sync_rejects_damping_motion_config_and_wrong_goal(self):
        original = self.fake.write2ByteTxRx
        for action in ("move-center", "jog"):
            for addr, value in ((40, 2), (58, 1), (66, 1), (31, 1), (33, 0), (42, 120)):
                with self.subTest(action=action, addr=addr):
                    self.fake.mem.update({40: 0, 56: 119, 57: 0, 58: 0, 66: 0, 31: 0, 33: 4})
                    port = self.byte_bus()
                    def alter(sid, address, target):
                        result = original(sid, address, target)
                        self.fake.mem[addr] = value
                        return result
                    with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=alter), \
                         self.assertRaises(servo.ServoError):
                        self.run_motion(action)
                    self.assertEqual([p[5] for p in port.packets if p[4] == 3], [42, 40])
                    self.assertEqual(self.fake.mem[40], 0)

    def range_case(self, start, low, high):
        self.fake.mem.update({40: 0, 56: start & 255, 57: start >> 8,
                              9: low & 255, 10: low >> 8, 11: high & 255, 12: high >> 8})
        return self.byte_bus()

    def test_center_pre_sync_range_rejects_small_but_invalid_feedback_before_goal(self):
        original = servo._record_result
        for start, invalid, low, high in ((0, 0x8001, 0, 4095), (4095, 4096, 0, 4095),
                                           (100, 99, 100, 3000), (3000, 3001, 100, 3000)):
            with self.subTest(start=start, invalid=invalid):
                port = self.range_case(start, low, high)
                def drift(path, event, data):
                    original(path, event, data)
                    if event == "pre_torque_guard":
                        self.fake.mem[56], self.fake.mem[57] = invalid & 255, invalid >> 8
                with mock.patch.object(servo, "_record_result", side_effect=drift), \
                     self.assertRaises(servo.ServoError):
                    self.run_motion("move-center")
                self.assertEqual([p for p in port.packets if p[4] == 3], [])
                self.assertEqual(self.fake.mem[40], 0)

    def test_center_post_sync_guard_retains_range_for_zero_and_one_torque(self):
        for torque in (0, 1):
            for start, invalid, low, high in ((0, 0x8001, 0, 4095), (4095, 4096, 0, 4095),
                                               (100, 99, 100, 3000), (3000, 3001, 100, 3000)):
                with self.subTest(torque=torque, start=start, invalid=invalid):
                    port = self.range_case(start, low, high)
                    before = self.bus.inspect(1)
                    self.fake.mem.update({40: torque, 42: start & 255, 43: start >> 8,
                                          56: invalid & 255, 57: invalid >> 8})
                    with self.assertRaises(servo.ServoError):
                        servo._motion_guard(self.bus, before, servo.MOVE_CENTER_TOLERANCE,
                                            goal_synced=True)
                    self.assertEqual([p for p in port.packets if p[4] == 3], [])

    def test_center_post_sync_invalid_range_cleans_up_without_explicit_enable(self):
        original = self.fake.write2ByteTxRx
        for auto in (False, True):
            for start, invalid, low, high in ((0, 0x8001, 0, 4095), (4095, 4096, 0, 4095),
                                               (100, 99, 100, 3000), (3000, 3001, 100, 3000)):
                with self.subTest(auto=auto, start=start, invalid=invalid):
                    port = self.range_case(start, low, high)
                    self.fake.goal_enables_torque = auto
                    def drift(sid, address, value):
                        result = original(sid, address, value)
                        self.fake.mem[56], self.fake.mem[57] = invalid & 255, invalid >> 8
                        return result
                    with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=drift), \
                         self.assertRaises(servo.ServoError):
                        self.run_motion("move-center")
                    writes = [p for p in port.packets if p[4] == 3]
                    self.assertEqual([p[5] for p in writes], [42, 40])
                    self.assertEqual([p[6] for p in writes if p[5] == 40], [0])
                    self.assertEqual(self.fake.mem[40], 0)

    def test_initial_goal_write_latency_counts_toward_total_deadline(self):
        original = self.fake.write2ByteTxRx
        def delayed(sid, addr, value):
            result = original(sid, addr, value)
            self.now += .2
            return result
        port = self.byte_bus()
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=delayed), \
             mock.patch.object(servo, "MOVE_CENTER_TOTAL_TIMEOUT_S", .1), \
             self.assertRaisesRegex(servo.ServoError, "总超时"):
            self.run_motion("move-center")
        self.assertEqual([p[5] for p in port.packets if p[4] == 3], [42, 40])
        self.assertEqual(self.fake.mem[40], 0)


class MotionMemoryPort:
    """字节级假设备：走真实 StrictHls 组包，不模拟电气、速度、回显或负载。"""
    is_using = False

    def __init__(self, device, *, drop_motion_ack=False):
        self.device = device
        self.drop_motion_ack = drop_motion_ack
        self.packets = []
        self.reply = bytearray()

    def getBytesAvailable(self):
        return len(self.reply)

    def readPort(self, length):
        chunk = bytes(self.reply[:length])
        del self.reply[:length]
        return chunk

    def writePort(self, data):
        packet = bytes(data)
        if packet[:2] != bytes([255, 255]) or packet[-1] != (~sum(packet[2:-1])) & 255:
            raise AssertionError("invalid frame")
        self.packets.append(packet)
        sid, inst = packet[2], packet[4]
        if sid != self.device.sid:
            raise AssertionError("unexpected id")
        payload = []
        if inst == 2:
            addr, size = packet[5:7]
            payload = [self.device.mem.get(i, 0) for i in range(addr, addr + size)]
        elif inst == 3:
            addr = packet[5]
            if len(packet) == 9 and addr == 42:
                powered = self.device.mem[40] == 1
                value = packet[6] | packet[7] << 8
                self.device.write2ByteTxRx(sid, addr, value)
                if powered and self.drop_motion_ack:
                    return len(data)  # 设备已执行但 ACK 丢失。
            elif len(packet) == 8 and addr == 40:
                self.device.write1ByteTxRx(sid, addr, packet[6])
            else:
                raise AssertionError("unexpected write")
        elif inst != 1:
            raise AssertionError("unexpected instruction")
        body = [sid, len(payload) + 2, 0, *payload]
        self.reply.extend([255, 255, *body, (~sum(body)) & 255])
        return len(data)


class AckPort:
    """字节级假端口：记录实际发出的帧，并回一个合法单播 ACK。不模拟电气与回显。"""
    is_using = False

    def __init__(self, sid, error=0):
        self.sid = sid
        self.error = error
        self.written = []
        self._pending = bytearray()

    def getBytesAvailable(self):
        return 0

    def writePort(self, packet):
        self.written.append(bytes(packet))
        body = [self.sid, 2, self.error]
        self._pending = bytearray([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])
        return len(packet)

    def readPort(self, n):
        chunk = bytes(self._pending[:n])
        del self._pending[:n]
        return chunk or None


class SdkPacketTests(unittest.TestCase):
    def test_write2_goal_frame_is_nine_bytes_and_reaches_port(self):
        from servo_toolkit.strict_sdk import StrictHls
        port = AckPort(sid=10)
        bus = servo.HlsBus(StrictHls(port), 0, -6)
        bus.write2(10, 42, 2046)
        self.assertEqual(len(port.written), 1)
        frame = port.written[0]
        self.assertEqual(len(frame), 9)
        self.assertEqual((frame[0], frame[1]), (0xFF, 0xFF))
        self.assertEqual(frame[2], 10)
        self.assertEqual(frame[3], 5)            # 参数长度 2 + 3
        self.assertEqual(frame[4], 0x03)         # INST_WRITE
        self.assertEqual(frame[5], 42)           # 目标位置寄存器
        self.assertEqual(frame[6] | frame[7] << 8, 2046)
        self.assertEqual(frame[8], (~sum(frame[2:8])) & 0xFF)

    def test_write2_to_other_address_is_never_sent(self):
        from servo_toolkit.strict_sdk import StrictHls
        port = AckPort(sid=10)
        bus = servo.HlsBus(StrictHls(port), 0, -6)
        with self.assertRaises(servo.CommunicationError):
            bus.write2(10, 31, 0)                # 偏移寄存器不得用双字节写
        self.assertEqual(port.written, [])
    def test_short_write_taints_and_does_not_retry(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        class Port:
            is_using = False
            def __init__(self):
                self.sent = 0
            def getBytesAvailable(self):
                return 0
            def writePort(self, packet):
                self.sent += 1
                return len(packet) - 1
        port = Port()
        bus = servo.HlsBus(StrictHls(port), 0, COMM_RX_TIMEOUT)
        with self.assertRaises(servo.CommunicationError):
            bus.read1(1, 40)
        self.assertTrue(port._microduck_tainted)
        self.assertFalse(port.is_using)
        self.assertEqual(port.sent, 1)
        with self.assertRaises(servo.CommunicationError):
            bus.read1(1, 40)
        self.assertEqual(port.sent, 1)

    def test_ping_status_is_not_overwritten_by_model_read(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        class Port:
            is_using = False
            def __init__(self):
                self.reply = bytearray()
                self.sent = []
            def getBytesAvailable(self):
                return len(self.reply)
            def writePort(self, packet):
                self.sent.append(bytes(packet))
                self.reply.extend(bytes.fromhex("FF FF 01 02 04 F8"))
                return len(packet)
            def readPort(self, length):
                value = self.reply[:length]
                del self.reply[:length]
                return bytes(value)
        port = Port()
        bus = servo.HlsBus(StrictHls(port), 0, COMM_RX_TIMEOUT)
        with self.assertRaises(servo.CommunicationError) as caught:
            bus.inspect(1)
        self.assertEqual(caught.exception.status, 4)
        self.assertEqual([packet[4] for packet in port.sent], [1])

    def test_continuous_noise_wrong_id_and_partial_frames_are_bounded(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        import time
        class Port:
            is_using = False
            def __init__(self, pattern):
                self.pattern = pattern
                self.sent = 0
                self.reads = 0
            def getBytesAvailable(self):
                return 0
            def writePort(self, packet):
                self.sent += 1
                return len(packet)
            def readPort(self, length):
                self.reads += 1
                if self.pattern == "noise":
                    return b"\x11" * length
                if self.pattern == "wrong":
                    return bytes.fromhex("FF FF 02 03 00 09 F1")[:length]
                if self.reads == 1:
                    return b"\xff"
                return b""
        for pattern in ("noise", "wrong", "partial"):
            with self.subTest(pattern=pattern):
                port = Port(pattern)
                bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, COMM_RX_TIMEOUT)
                start = time.monotonic()
                with self.assertRaises(servo.CommunicationError) as exc:
                    bus.read1(1, 40)
                self.assertEqual(exc.exception.result, -7)
                self.assertLess(time.monotonic() - start, 0.2)
                self.assertTrue(port._microduck_tainted)
                self.assertEqual(port.sent, 1)

    def test_busy_belongs_to_other_transaction_and_is_never_cleared(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_PORT_BUSY
        class Port:
            is_using = True
            def getBytesAvailable(self):
                raise AssertionError("busy 时不得访问串口")
        port = Port()
        handler = StrictHls(port)
        _, code, _ = handler.ping(1)
        self.assertEqual(code, COMM_PORT_BUSY)
        self.assertTrue(port.is_using)
        self.assertTrue(handler._transaction_lock.acquire(blocking=False))
        try:
            _, code, _ = handler.ping(1)
            self.assertEqual(code, COMM_PORT_BUSY)
            self.assertTrue(port.is_using)
        finally:
            handler._transaction_lock.release()

    def test_existing_or_delayed_wrong_id_is_not_a_clean_timeout(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        class Port:
            is_using = False
            def __init__(self):
                self.reply = bytearray()
                self.sent = 0
            def getBytesAvailable(self):
                return len(self.reply)
            def writePort(self, packet):
                self.sent += 1
                self.reply.extend(bytes.fromhex("FF FF 01 03 00 01 FA"))
                return len(packet)
            def readPort(self, length):
                value = self.reply[:length]
                del self.reply[:length]
                return value
        port = Port()
        bus = servo.HlsBus(StrictHls(port), 0, COMM_RX_TIMEOUT)
        with self.assertRaises(servo.CommunicationError) as caught:
            bus.probe(10)
        self.assertEqual(caught.exception.result, -7)
        with self.assertRaises(servo.CommunicationError):
            bus.read1(1, 5)
        self.assertEqual(port.sent, 1)

    def test_strict_sdk_rejects_bad_lengths_checksums_and_wrong_id(self):
        try:
            from servo_toolkit.strict_sdk import StrictHls
            from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        except ImportError as exc:
            self.skipTest(f"pyserial 未安装：{exc}")
        import time

        def frame(sid, data=(), *, error=0):
            body = [sid, len(data) + 2, error, *data]
            return bytes([255, 255, *body, (~sum(body)) & 255])

        class MemoryPort:
            is_using = False
            def __init__(self, response):
                self.response = response
                self.reply = bytearray()
                self.deadline = 0
            def getBytesAvailable(self):
                return len(self.reply)
            def clearPort(self):
                self.reply.clear()
            def setPacketTimeout(self, length):
                self.deadline = time.monotonic() + 0.002
            def isPacketTimeout(self):
                return time.monotonic() > self.deadline
            def readPort(self, length):
                result = self.reply[:length]
                del self.reply[:length]
                return bytes(result)
            def writePort(self, data):
                self.reply.extend(self.response)
                return len(data)

        malformed = [
            ("zero data as read1", frame(253), 253, 1),
            ("extra data as read1", frame(1, [1, 2]), 1, 1),
            ("short data as read2", frame(1, [7]), 1, 2),
            ("extra data as read2", frame(1, [1, 2, 3]), 1, 2),
            ("wrong id", frame(2, [0]), 1, 1),
            ("bad checksum", frame(1, [0])[:-1] + b"\x00", 1, 1),
        ]
        for label, response, sid, length in malformed:
            with self.subTest(label=label):
                port = MemoryPort(response)
                bus = servo.HlsBus(StrictHls(port), 0, COMM_RX_TIMEOUT)
                with self.assertRaises(servo.CommunicationError):
                    (bus.read1 if length == 1 else bus.read2)(sid, 40)
                self.assertFalse(port.is_using)
        # 写 ACK 多带参数不得当成功。
        port = MemoryPort(frame(1, [0]))
        bus = servo.HlsBus(StrictHls(port), 0, COMM_RX_TIMEOUT)
        with self.assertRaises(servo.CommunicationError):
            bus.write1(1, 55, 1)
        # 正确回复仍可正常解析。
        bus = servo.HlsBus(StrictHls(MemoryPort(frame(1, [7]))), 0, COMM_RX_TIMEOUT)
        self.assertEqual(bus.read1(1, 40), 7)

    def test_two_wrappers_sharing_port_do_not_steal_busy(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_PORT_BUSY
        import threading
        class Port:
            is_using = False
            def __init__(self):
                self.started = threading.Event()
                self.release = threading.Event()
                self.sent = 0
            def getBytesAvailable(self):
                return 0
            def writePort(self, data):
                self.sent += 1
                self.started.set()
                self.release.wait(1)
                return len(data)
            def readPort(self, n):
                return b""
        port = Port()
        first = StrictHls(port, timeout_seconds=.005)
        second = StrictHls(port, timeout_seconds=.005)
        done = []
        worker = threading.Thread(target=lambda: done.append(first.ping(1)))
        worker.start()
        self.assertTrue(port.started.wait(1))
        self.assertTrue(port.is_using)
        _, code, _ = second.ping(1)
        self.assertEqual(code, COMM_PORT_BUSY)
        self.assertTrue(port.is_using)
        self.assertEqual(port.sent, 1)
        port.release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertFalse(port.is_using)
        self.assertEqual(len(done), 1)

    def test_sdk_transport_exception_resets_busy_but_does_not_retry(self):
        try:
            from servo_toolkit.strict_sdk import StrictHls
        except ImportError as exc:
            self.skipTest(f"pyserial 未安装：{exc}")
        class BrokenPort:
            is_using = False
            calls = 0
            def clearPort(self):
                pass
            def getBytesAvailable(self):
                return 0
            def writePort(self, data):
                self.calls += 1
                raise OSError("send may have partially succeeded")
        port = BrokenPort()
        handler = StrictHls(port)
        with self.assertRaises(OSError):
            handler.write1ByteTxRx(1, 55, 0)
        self.assertFalse(port.is_using)
        self.assertEqual(port.calls, 1)

    def test_vendored_sdk_calibrate_frame_and_readback(self):
        try:
            from servo_toolkit.vendor.scservo_sdk import hls, COMM_SUCCESS, COMM_RX_TIMEOUT
        except ImportError as exc:
            self.skipTest(f"pyserial 未安装：{exc}")

        class MemoryPort:
            is_using = False
            def __init__(self):
                self.tx = []
            def clearPort(self):
                pass
            def writePort(self, data):
                self.tx.append(bytes(data))
                return len(data)
            def setPacketTimeout(self, length):
                pass
            def readPort(self, length):
                raise AssertionError("此测试不应执行真实或模拟串口读取")

        port = MemoryPort()
        handler = hls(port)
        # 用假的响应层保留官方组包过程；只验证 SDK 生成字节，不做硬件测试。
        def fake_exchange(packet):
            rc = handler.txPacket(packet)
            port.is_using = False
            return [255, 255, packet[2], 2, 0, 0], rc, 0
        handler.txRxPacket = fake_exchange
        bus = servo.HlsBus(handler, COMM_SUCCESS, COMM_RX_TIMEOUT)
        bus.calibrate(1)
        self.assertEqual(port.tx, [bytes.fromhex("FF FF 01 04 0B 00 08 E7")])

    def test_relock_ack_timeout_stops_and_gives_safe_power_off_guidance(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT

        class Port:
            is_using = False
            def __init__(self):
                self.sent = []
                self.reply = bytearray()
                self.lock = 1
            def getBytesAvailable(self):
                return len(self.reply)
            def writePort(self, packet):
                self.sent.append(bytes(packet))
                sid, instruction = packet[2], packet[4]
                data = []
                if instruction == 3 and packet[5] == 55:
                    self.lock = packet[6]
                    if self.lock == 1:  # 假设备锁已生效，但不回复 ACK。
                        return len(packet)
                elif instruction == 2 and packet[5] == 55:
                    data = [self.lock]
                else:
                    raise AssertionError(f"unexpected packet: {packet}")
                body = [sid, len(data) + 2, 0, *data]
                self.reply.extend([255, 255, *body, (~sum(body)) & 255])
                return len(packet)
            def readPort(self, length):
                data = self.reply[:length]
                del self.reply[:length]
                return bytes(data)

        port = Port()
        bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, COMM_RX_TIMEOUT)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "record.jsonl"
            path.write_text("", encoding="utf-8")
            with self.assertRaises(servo.ServoError) as caught:
                servo._unlocked_operation(bus, 1, None, lambda _: None, path)
        error = str(caught.exception)
        self.assertIn("锁状态未知", error)
        self.assertIn("关闭外部电源", error)
        self.assertIn("不要带电插拔", error)
        self.assertEqual(port.lock, 1)  # 只是假设备的内部状态，软件不得依此报成功。
        self.assertTrue(port._microduck_tainted)
        self.assertEqual([(p[4], p[5]) for p in port.sent], [(3, 55), (2, 55), (3, 55)])

    def test_unlock_ack_lost_stops_without_sending_id_or_relock(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        class Port:
            is_using = False
            def __init__(self):
                self.sent = []
                self.lock = 1
            def getBytesAvailable(self):
                return 0
            def writePort(self, packet):
                self.sent.append(bytes(packet))
                if packet[4:7] == [3, 55, 0]:
                    self.lock = 0  # 模拟设备执行，但 ACK 丢失。
                return len(packet)
            def readPort(self, length):
                return b""
        port = Port()
        bus = servo.HlsBus(StrictHls(port, timeout_seconds=.01), 0, COMM_RX_TIMEOUT)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "record.jsonl"
            path.write_text("", encoding="utf-8")
            with self.assertRaisesRegex(servo.ServoError, "锁状态未知"):
                servo._unlocked_operation(bus, 1, 10,
                    lambda _: self.fail("不得发送 ID 写入"), path)
        self.assertEqual(port.lock, 0)
        self.assertEqual(len(port.sent), 1)
        self.assertEqual(port.sent[0][2:7], bytes([1, 4, 3, 55, 0]))
        self.assertTrue(port._microduck_tainted)

    def test_byte_level_id_write_can_read_new_id_and_lock(self):
        """仅假端口且 mock profile：写主 ID 后回读并重锁新 ID。"""
        try:
            from servo_toolkit.vendor.scservo_sdk import hls, COMM_SUCCESS, COMM_RX_TIMEOUT
        except ImportError as exc:
            self.skipTest(f"pyserial 未安装：{exc}")
        import time

        class MemoryPort:
            is_using = False
            def __init__(self):
                self.device = FakeHls()
                self.reply = bytearray()
                self.deadline = 0
                self.packets = []
            def getBytesAvailable(self):
                return len(self.reply)
            def clearPort(self):
                self.reply.clear()
            def setPacketTimeout(self, length):
                self.deadline = time.monotonic() + 0.003
            def isPacketTimeout(self):
                return time.monotonic() > self.deadline
            def readPort(self, length):
                data = self.reply[:length]
                del self.reply[:length]
                return bytes(data)
            def writePort(self, data):
                packet = list(data)
                self.packets.append(packet)
                self.assert_packet(packet)
                sid, inst = packet[2], packet[4]
                if sid != self.device.sid:
                    return len(data)
                mem = self.device.mem
                payload = []
                if inst == 2:
                    addr, size = packet[5:7]
                    payload = [mem.get(i, 0) for i in range(addr, addr + size)]
                elif inst == 3:
                    addr, value = packet[5:7]
                    mem[addr] = value
                    if addr == 5:
                        self.device.sid = value
                        sid = value  # 新 ID 回 ACK：官方 SDK 将等旧 ID，直至超时。
                elif inst != 1:
                    raise AssertionError(f"Unexpected instruction {inst}")
                response = [sid, len(payload) + 2, 0, *payload]
                checksum = (~sum(response)) & 255
                self.reply.extend([255, 255, *response, checksum])
                return len(data)
            @staticmethod
            def assert_packet(packet):
                if packet[:2] != [255, 255] or packet[-1] != ((~sum(packet[2:-1])) & 255):
                    raise AssertionError(f"bad SDK packet: {packet}")

        from servo_toolkit.strict_sdk import StrictHls
        port = MemoryPort()
        bus = servo.HlsBus(StrictHls(port), COMM_SUCCESS, COMM_RX_TIMEOUT)
        before = bus.inspect(1)
        # 模型默认对新 ID 回 ACK；此分支为模拟，不代表真实舵机行为。
        # inspect 全程只能发单播 PING/READ；不调用任何写入或广播。
        self.assertTrue(all(p[2] == 1 and p[4] in (1, 2) for p in port.packets))
        with tempfile.TemporaryDirectory() as tmp, \
             mock.patch.object(servo, "_require_validated_profile"):
            result = servo.change_id(bus, port="fake", expected=before, new_id=10,
                confirmation=servo.CONFIRM_ID, records=Path(tmp))
        self.assertEqual(result["new_id"], 10)
        self.assertEqual(port.device.sid, 10)
        self.assertEqual(port.device.mem[55], 1)
        self.assertTrue(any(p[2] == 1 and p[4:7] == [3, 5, 10] for p in port.packets))
        self.assertFalse(any(p[2] == 254 for p in port.packets))
        self.assertFalse(any(p[4] == 3 and p[5] in (40, 42, 43) for p in port.packets))


class SetIdCliTests(unittest.TestCase):
    """仅假设备：CLI set-id 不读 stdin、不查 TTY；真实 change_id 护栏仍生效。"""
    def setUp(self):
        import servo_toolkit.__main__ as cli
        self.cli = cli
        self.fake = FakeHls()
        self.fake.mem[4] = 31  # 命中真实 profile，不 mock 型号/状态门禁。
        self.bus = servo.HlsBus(self.fake, 0, -6)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.records = Path(self.tmp.name) / "records"

    def run_set_id(self, new_id=10, *, stdin_tty=None, stdout_tty=None):
        # None => isatty 一旦被调用便抛异常；False 单独模拟输入/输出重定向。
        stdin, stdout, self.stderr = io.StringIO(), io.StringIO(), io.StringIO()
        def tty(value):
            return ({"side_effect": AssertionError("set-id 不得查询 isatty")}
                    if value is None else {"return_value": value})
        with mock.patch.object(self.cli, "connect") as connect, \
             mock.patch.object(self.cli, "RECORDS", self.records), \
             mock.patch("sys.stdin", stdin), contextlib.redirect_stdout(stdout), \
             contextlib.redirect_stderr(self.stderr), \
             mock.patch.object(stdin, "isatty", **tty(stdin_tty)) as in_tty, \
             mock.patch.object(stdout, "isatty", **tty(stdout_tty)) as out_tty, \
             mock.patch.object(stdin, "read", side_effect=AssertionError("不得读取 stdin")) as read, \
             mock.patch.object(stdin, "readline", side_effect=AssertionError("不得读取 stdin")) as line, \
             mock.patch("builtins.input", side_effect=AssertionError("set-id 不得调用 input")) as prompt:
            connect.return_value.__enter__.return_value = self.bus
            rc = self.cli.main(["--port", "fake", "set-id", "--id", "1", "--new-id", str(new_id)])
        for forbidden in (in_tty, out_tty, read, line, prompt):
            forbidden.assert_not_called()
        self.connect = connect
        return rc

    def writes(self):
        return [c for c in self.fake.calls if c[0] in ("write", "write2", "calibrate")]

    def rows(self):
        files = list(self.records.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        return [json.loads(line) for line in files[0].read_text().splitlines()]

    def test_set_id_never_queries_tty_or_reads_input(self):
        self.assertEqual(self.run_set_id(), 0)
        self.connect.assert_called_once_with("fake", baud=1_000_000)
        self.assertEqual(self.writes(), [("write", 1, 55), ("write", 1, 5), ("write", 10, 55)])
        self.assertEqual((self.fake.sid, self.fake.mem[55], self.fake.mem[40]), (10, 1, 0))
        self.assertEqual([r["event"] for r in self.rows()], ["before", "before_unlock", "unlock_confirmed",
                         "before_id_write", "before_relock", "relock_confirmed", "after_read", "result"])

    def test_set_id_non_tty_stdin_does_not_require_stdout_tty(self):
        self.assertEqual(self.run_set_id(stdin_tty=False), 0)
        self.assertEqual((self.fake.sid, self.fake.mem[55]), (10, 1))

    def test_set_id_non_tty_stdout_does_not_require_stdin_tty(self):
        self.assertEqual(self.run_set_id(stdout_tty=False), 0)
        self.assertEqual((self.fake.sid, self.fake.mem[55]), (10, 1))

    def test_set_id_legal_endpoint_ids(self):
        for target in (0, 253):
            with self.subTest(target=target):
                self.fake.sid = self.fake.mem[5] = 1
                self.assertEqual(self.run_set_id(target), 0)
                self.assertEqual((self.fake.sid, self.fake.mem[55]), (target, 1))

    def test_set_id_invalid_ids_reject_before_connect(self):
        for flag in ("--id", "--new-id"):
            for invalid in ("-1", "254", "255", "1.5", "True"):
                argv = ["--port", "fake", "set-id", "--id", "1", "--new-id", "10"]
                argv[argv.index(flag) + 1] = invalid
                with self.subTest(flag=flag, invalid=invalid), \
                     mock.patch.object(self.cli, "connect") as connect, \
                     mock.patch("sys.stdin.isatty", side_effect=AssertionError("不应查询 TTY")), \
                     mock.patch("sys.stdout.isatty", side_effect=AssertionError("不应查询 TTY")), \
                     mock.patch("builtins.input", side_effect=AssertionError("不应确认")), \
                     contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                    self.cli.main(argv)
                self.assertEqual(caught.exception.code, 2)
                connect.assert_not_called()
        self.assertEqual(self.writes(), [])

    def test_set_id_same_id_rejects_without_writes_or_records(self):
        self.assertEqual(self.run_set_id(1), 1)
        self.assertIn("新旧 ID 相同", self.stderr.getvalue())
        self.assertEqual(self.writes(), [])
        self.assertFalse(self.records.exists())

    def test_set_id_writes_disabled_rejects_before_connect(self):
        with mock.patch.object(self.cli, "WRITES_ENABLED", False):
            self.assertEqual(self.run_set_id(), 2)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])
        self.assertFalse(self.records.exists())

    def test_change_id_still_requires_exact_confirmation(self):
        before = self.bus.inspect(1)
        self.fake.calls.clear()
        for confirmation in ("", "yes", servo.CONFIRM_ID + " "):
            with self.subTest(confirmation=confirmation), self.assertRaisesRegex(servo.ServoError, "确认语"):
                servo.change_id(self.bus, port="fake", expected=before, new_id=10,
                                confirmation=confirmation, records=self.records)
        self.assertEqual(self.fake.calls, [])
        self.assertFalse(self.records.exists())

    def test_set_id_real_profile_and_state_preconditions_still_reject(self):
        original = self.fake.mem.copy()
        for addr, value in ((4, 2), (2, 1), (6, 3), (8, 0), (55, 0), (40, 1),
                            (58, 1), (66, 1), (65, 1), (62, 39), (32, 0x70)):
            with self.subTest(addr=addr, value=value):
                self.fake.mem = {**original, addr: value}
                self.fake.calls.clear()
                self.assertEqual(self.run_set_id(), 1)
                self.assertEqual(self.writes(), [])
                self.assertFalse(self.records.exists())

    def test_set_id_record_fsync_failure_stops_before_unlock(self):
        with mock.patch.object(servo.os, "fsync", side_effect=OSError("disk sync")) as sync:
            self.assertEqual(self.run_set_id(), 1)
        sync.assert_called_once()
        self.assertIn("disk sync", self.stderr.getvalue())
        self.assertEqual(self.writes(), [])
        self.assertEqual(self.fake.mem[55], 1)

    def test_set_id_write_stages_are_fsynced_before_packets(self):
        events, synced = [], []
        record, start, sync, write = servo._record_result, servo._record_start, servo.os.fsync, self.fake.write1ByteTxRx
        def fsync(fd):
            sync(fd)
            synced.append(fd)
        def begin(*args):
            count = len(synced)
            path = start(*args)
            self.assertGreaterEqual(len(synced) - count, 3)  # 父目录、文件、records目录
            events.append("before")
            return path
        def stage(path, event, data):
            count = len(synced)
            record(path, event, data)
            self.assertGreater(len(synced), count)
            events.append(event)
        def packet(sid, addr, value):
            expected = {(55, 0): "before_unlock", (5, 10): "before_id_write", (55, 1): "before_relock"}
            self.assertEqual(events[-1], expected[(addr, value)])
            return write(sid, addr, value)
        with mock.patch.object(servo.os, "fsync", side_effect=fsync), \
             mock.patch.object(servo, "_record_start", side_effect=begin), \
             mock.patch.object(servo, "_record_result", side_effect=stage), \
             mock.patch.object(self.fake, "write1ByteTxRx", side_effect=packet):
            self.assertEqual(self.run_set_id(), 0)
        self.assertEqual(events[-1], "result")

    def test_set_id_actual_change_id_failures_stop_without_retry(self):
        cases = (("unlock", ("write", 1, 55), [-6, None], 1, 1, [55, 55]),
                 ("id_ack", None, None, 10, 0, [55, 5]),
                 ("id_readback", ("read1", 10, 5), -6, 10, 0, [55, 5]),
                 ("relock", ("write", 10, 55), -6, 10, 0, [55, 5, 55]))
        for name, key, fault, sid, lock, addresses in cases:
            with self.subTest(name=name):
                self.fake = FakeHls()
                self.fake.mem[4] = 31
                self.bus = servo.HlsBus(self.fake, 0, -6)
                self.records = Path(self.tmp.name) / name
                if key is None:
                    self.fake.id_timeout_after_apply = True
                else:
                    self.fake.fail[key] = fault
                self.assertEqual(self.run_set_id(), 1)
                self.assertEqual((self.fake.sid, self.fake.mem[55]), (sid, lock))
                self.assertEqual([c[2] for c in self.writes()], addresses)
                self.assertEqual(self.rows()[-1]["event"], "error")

    def test_set_id_strict_hls_id_ack_and_taint_boundaries(self):
        from servo_toolkit.strict_sdk import StrictHls
        from servo_toolkit.vendor.scservo_sdk import COMM_RX_TIMEOUT
        class IdPort:
            is_using = False
            def __init__(port, device, fault):
                port.device, port.fault = device, fault
                port.packets, port.reply = [], bytearray()
            def getBytesAvailable(port):
                return len(port.reply)
            def readPort(port, length):
                data = bytes(port.reply[:length])
                del port.reply[:length]
                return data
            def writePort(port, data):
                packet = bytes(data)
                if packet[:2] != b"\xff\xff" or packet[-1] != (~sum(packet[2:-1])) & 255:
                    raise AssertionError("invalid packet")
                port.packets.append(packet)
                sid, inst = packet[2], packet[4]
                if sid != port.device.sid:
                    raise AssertionError("wrong ID or scan")
                payload = []
                if inst == 2:
                    addr, size = packet[5:7]
                    payload = [port.device.mem.get(i, 0) for i in range(addr, addr + size)]
                elif inst == 3:
                    addr, value = packet[5:7]
                    if addr not in (5, 55) or len(packet) != 8:
                        raise AssertionError("set-id must not write motion/other registers")
                    port.device.write1ByteTxRx(sid, addr, value)
                    if addr == 5:
                        if port.fault in ("lost", "short"):
                            return len(data) - (port.fault == "short")
                        if port.fault == "new_ack":
                            sid = value
                elif inst != 1:
                    raise AssertionError("unexpected instruction")
                body = [sid, len(payload) + 2, 0, *payload]
                port.reply.extend([255, 255, *body, (~sum(body)) & 255])
                return len(data)
        for fault in ("old_ack", "new_ack", "lost", "short"):
            with self.subTest(fault=fault):
                self.fake = FakeHls()
                self.fake.mem[4] = 31
                port = IdPort(self.fake, fault)
                self.bus = servo.HlsBus(StrictHls(port, timeout_seconds=.005), 0, COMM_RX_TIMEOUT)
                self.records = Path(self.tmp.name) / fault
                uncertain = fault in ("lost", "short")
                self.assertEqual(self.run_set_id(), 1 if uncertain else 0)
                self.assertEqual((self.fake.sid, self.fake.mem[55]), (10, 0 if uncertain else 1))
                self.assertEqual([c[2] for c in self.writes()], [55, 5] if uncertain else [55, 5, 55])
                if uncertain:
                    self.assertTrue(port._microduck_tainted)
                    self.assertEqual(port.packets[-1][2:7], bytes([1, 4, 3, 5, 10]))
                    self.assertIn("关闭外部电源", self.stderr.getvalue())
                    self.assertEqual(self.rows()[-1]["event"], "error")
                else:
                    self.assertEqual(self.rows()[-1]["event"], "result")

    def test_non_set_id_commands_keep_tty_and_confirmation(self):
        commands = (("calibrate-center", "calibrate_center", servo.CONFIRM_CENTER),
                    ("jog", "jog_test", servo.CONFIRM_JOG),
                    ("move-center", "move_center", servo.CONFIRM_MOVE_CENTER))
        # 仅测试时隔离更早的CLI动作门禁；产品四道门禁源码仍False。
        with mock.patch.object(self.cli, "JOG_ENABLED", True), \
             mock.patch.object(self.cli, "MOVE_CENTER_ENABLED", True):
            for command, operation, phrase in commands:
                for in_tty, out_tty in ((False, True), (True, False)):
                    with self.subTest(command=command, stdin=in_tty, stdout=out_tty), \
                         mock.patch.object(self.cli, "connect") as connect, \
                         mock.patch("sys.stdin.isatty", return_value=in_tty), \
                         mock.patch("sys.stdout.isatty", return_value=out_tty), \
                         mock.patch("builtins.input", side_effect=AssertionError("TTY拒绝应早于提示")), \
                         contextlib.redirect_stderr(io.StringIO()):
                        self.assertEqual(self.cli.main(["--port", "fake", command, "--id", "1"]), 2)
                    connect.assert_not_called()
                with self.subTest(command=command, tty=True), \
                     mock.patch.object(self.cli, "connect") as connect, \
                     mock.patch.object(self.cli, operation, return_value={}) as action, \
                     contextlib.redirect_stdout(io.StringIO()), \
                     mock.patch("sys.stdin.isatty", return_value=True), \
                     mock.patch("sys.stdout.isatty", return_value=True), \
                     mock.patch("builtins.input", return_value="caller supplied") as prompt:
                    connect.return_value.__enter__.return_value = self.bus
                    self.assertEqual(self.cli.main(["--port", "fake", command, "--id", "1"]), 0)
                prompt.assert_called_once_with(f"输入「{phrase}」确认，否则取消：")
                self.assertEqual(action.call_args.kwargs["confirmation"], "caller supplied")
        self.assertEqual(self.writes(), [])

    def test_default_motion_and_baud_gates_stay_closed_before_connect(self):
        self.assertFalse(self.cli.JOG_ENABLED)
        self.assertFalse(self.cli.MOVE_CENTER_ENABLED)
        self.assertFalse(servo.JOG_ARMED)
        self.assertFalse(servo.MOVE_CENTER_ARMED)
        self.assertTrue(self.cli.WRITES_ENABLED)
        for command in ("jog", "move-center", "set-baud"):
            with self.subTest(command=command), mock.patch.object(self.cli, "connect") as connect, \
                 mock.patch("sys.stdin.isatty", side_effect=AssertionError("不应查询TTY")), \
                 mock.patch("sys.stdout.isatty", side_effect=AssertionError("不应查询TTY")), \
                 mock.patch("builtins.input", side_effect=AssertionError("不应提示")), \
                 contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(self.cli.main(["--port", "fake", command, "--id", "1"]), 2)
            connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
