"""Isolated fake-device tests; no TTY, real serial or gate mutation."""
import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from servo_toolkit import motion, servo
from servo_toolkit import __main__ as cli
from servo_toolkit.tests.test_servo import FakeHls, MotionMemoryPort


class MotionEntryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.records = Path(self.tmp.name) / "records"
        self.port = "/offline/C001"
        self.node = {"path": self.port, "dev": 1, "ino": 2, "rdev": 3, "ctime_ns": 4}
        self.fake = FakeHls()
        self.fake.mem.update({4: 31, 7: 253, 18: 52, 33: 4, 56: 24, 57: 0})
        self.fake.goal_enables_torque = True
        self.bus = servo.HlsBus(self.fake, 0, -6)
        self.now = 1000.0
        for patch in (mock.patch.object(motion, "_device", side_effect=lambda p: dict(self.node)),
                      mock.patch.object(servo.time, "time", side_effect=lambda: self.now),
                      mock.patch.object(servo.time, "monotonic", side_effect=lambda: self.now),
                      mock.patch.object(servo.time, "sleep", side_effect=self.sleep)):
            patch.start()
            self.addCleanup(patch.stop)
        @contextlib.contextmanager
        def connect(*args, **kwargs):
            yield self.bus
        patch = mock.patch.object(servo, "connect", side_effect=connect)
        self.connect = patch.start()
        self.addCleanup(patch.stop)

    def sleep(self, seconds):
        self.now += max(seconds, 0.001)

    def position(self, value):
        self.fake.mem[56], self.fake.mem[57] = value & 255, value >> 8

    def prepare(self, scope="center-jog50"):
        result = motion.prepare(port=self.port, sid=1, scope=scope, baud=servo.BAUD, records=self.records)
        self.fake.calls.clear()
        self.connect.reset_mock()
        return result

    def execute(self, plan, **overrides):
        report = {"stage": "validation", "center": {"status": "not_run"},
                  "jog50": {"status": "not_run"}, "final_inspect": {"status": "not_run"}}
        options = dict(port=self.port, sid=1, scope=plan["scope"], baud=servo.BAUD,
                       records=self.records, plan_id=plan["plan_id"],
                       approval=plan["approve_sha256"], report=report)
        options.update(overrides)
        return motion.run(**options)

    def writes(self):
        return [call for call in self.fake.calls if call[0] in ("write", "write2", "calibrate")]

    def cli_run(self, plan):
        output = io.StringIO()
        with mock.patch.object(cli, "RECORDS", self.records), contextlib.redirect_stdout(output):
            code = cli.main(["--port", self.port, "motion-run", "--id", "1", "--scope", plan["scope"],
                             "--plan", plan["plan_id"], "--approve", plan["approve_sha256"]])
        return code, json.loads(output.getvalue())

    def test_prepare_read_only_and_records_are_new(self):
        old = self.records / "old.jsonl"
        self.records.mkdir()
        old.write_text("unchanged\n")
        plan = self.prepare()
        self.assertEqual(self.writes(), [])
        self.assertEqual(plan["position"], 24)
        self.assertEqual(plan["snapshot"]["baud_index"], 0)
        self.assertEqual(old.read_text(), "unchanged\n")
        self.assertEqual(len(list((self.records / ".motion-plans").glob("*.json"))), 1)
        self.assertFalse(servo.MOVE_CENTER_ARMED or servo.JOG_ARMED)
        self.assertFalse(cli.MOVE_CENTER_ENABLED or cli.JOG_ENABLED)

    def test_both_motion_paths_reuse_real_functions_and_keep_four_gates_false(self):
        for auto in (False, True):
            with self.subTest(automatic_enable=auto):
                self.position(24)
                self.fake.goal_enables_torque = auto
                plan = self.prepare()
                with mock.patch("builtins.input", side_effect=AssertionError("no input")), \
                     mock.patch("sys.stdin.isatty", side_effect=AssertionError("no tty")), \
                     mock.patch("sys.stdout.isatty", side_effect=AssertionError("no tty")):
                    report = self.execute(plan)
                self.assertEqual(report["exit_code"], 0)
                self.assertEqual(report["center"]["status"], "passed")
                self.assertEqual(report["jog50"]["result"]["amplitude"], 50)
                self.assertEqual(report["final_inspect"]["snapshot"]["position"], 2048)
                self.assertEqual(self.fake.mem[40], 0)
                self.assertTrue(all(call[2] in (40, 42) for call in self.writes()))
                self.assertFalse(servo.MOVE_CENTER_ARMED or servo.JOG_ARMED)
                self.assertFalse(cli.MOVE_CENTER_ENABLED or cli.JOG_ENABLED)

    def test_expiry_during_one_second_wait_stops_before_next_goal(self):
        self.position(2000)
        plan = self.prepare()
        self.now += motion.PLAN_TTL_S - 0.5
        code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(report["center"]["status"], "failed_or_uncertain")
        self.assertEqual(report["jog50"]["status"], "not_run")
        self.assertIn("过期", report["error"])
        self.assertEqual(self.writes(), [("write2", 1, 42), ("write2", 1, 42),
                                         ("write", 1, 40)])
        self.assertEqual(self.fake.mem[56] | self.fake.mem[57] << 8, 2032)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertTrue((self.records / ".motion-plans" / (plan["plan_id"] + ".used")).is_file())
        self.fake.calls.clear()
        with self.assertRaisesRegex(servo.ServoError, "过期|消费"):
            self.execute(plan)
        self.assertEqual(self.writes(), [])

    def test_center_only_in_window_never_writes_or_jogs(self):
        self.position(2050)
        plan = self.prepare("center")
        with mock.patch.object(servo, "jog_test") as jog:
            report = self.execute(plan)
        jog.assert_not_called()
        self.assertEqual(self.writes(), [])
        self.assertEqual(report["jog50"]["status"], "not_run")
        self.assertEqual(list(self.records.glob("*.jsonl")), [])

    def test_prepare_rejects_bad_profile_and_unsafe_state(self):
        for address, value in ((4, 2), (1, 45), (33, 0), (40, 1), (55, 0), (6, 3), (58, 1)):
            with self.subTest(address=address):
                old = self.fake.mem[address]
                self.fake.mem[address] = value
                with self.assertRaises(servo.ServoError):
                    self.prepare()
                self.fake.mem[address] = old
        self.assertEqual(self.writes(), [])
        self.assertFalse(self.records.exists())

    def test_missing_parameters_and_wrong_types_reject_before_connect(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for suffix in ([], ["--plan", "a" * 32]):
                with self.assertRaises(SystemExit):
                    cli.main(["--port", self.port, "motion-run", "--id", "1", "--scope", "center", *suffix])
        self.connect.assert_not_called()
        self.assertFalse(self.records.exists())

    def test_wrong_plan_approval_id_port_scope_and_baud_before_connect(self):
        plan = self.prepare()
        for overrides in ({"approval": "b" * 64}, {"plan_id": "../../escape"},
                          {"sid": 2}, {"scope": "center"}, {"port": "/offline/other"},
                          {"baud": 500000}):
            with self.subTest(overrides=overrides), self.assertRaises(servo.ServoError):
                self.execute(plan, **overrides)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])
        self.assertEqual(list((self.records / ".motion-plans").glob("*.used")), [])

    def test_expired_or_clock_rollback_rejects_before_connect(self):
        plan = self.prepare()
        for now in (999, 1300, 1400):
            self.now = now
            with self.assertRaisesRegex(servo.ServoError, "过期|时间无效"):
                self.execute(plan)
        self.connect.assert_not_called()
        self.assertEqual(self.writes(), [])

    def test_source_or_device_changed_invalidates_plan_before_connect(self):
        plan = self.prepare()
        with mock.patch.object(motion, "source_sha256", return_value="changed"), \
             self.assertRaisesRegex(servo.ServoError, "源码/设备"):
            self.execute(plan)
        self.node["ino"] += 1
        with self.assertRaisesRegex(servo.ServoError, "源码/设备"):
            self.execute(plan)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])

    def test_plan_tampering_refused_before_connect(self):
        plan = self.prepare()
        path = Path(plan["plan_record"])
        data = json.loads(path.read_text())
        data["snapshot"]["position"] = 24 + 1
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(servo.ServoError, "摘要"):
            self.execute(plan)
        self.connect.assert_not_called()

    def test_replay_refused_before_connect_after_success(self):
        self.position(2048)
        plan = self.prepare("center")
        self.execute(plan)
        self.connect.reset_mock()
        self.fake.calls.clear()
        with self.assertRaisesRegex(servo.ServoError, "已消费"):
            self.execute(plan)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])

    def test_failed_connection_also_burns_plan(self):
        plan = self.prepare()
        self.connect.side_effect = OSError("serial unavailable")
        with self.assertRaises(OSError):
            self.execute(plan)
        self.connect.reset_mock()
        with self.assertRaisesRegex(servo.ServoError, "已消费"):
            self.execute(plan)
        self.connect.assert_not_called()

    def test_config_or_position_change_after_prepare_blocks_motion(self):
        for address in (31, 56):
            plan = self.prepare()
            self.fake.mem[address] += 8
            with self.assertRaises(servo.ServoError):
                self.execute(plan)
            self.fake.mem[address] -= 8
        self.assertEqual(self.writes(), [])

    def test_consume_record_fsync_failure_does_not_connect(self):
        plan = self.prepare()
        with mock.patch.object(servo, "_sync_dir", side_effect=OSError("fsync")), \
             self.assertRaises(OSError):
            self.execute(plan)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])

    def test_center_failure_never_calls_jog_and_has_stage_report(self):
        self.fake.no_motion = True
        plan = self.prepare()
        with mock.patch.object(servo, "jog_test") as jog:
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(report["center"]["status"], "failed_or_uncertain")
        self.assertEqual(report["jog50"]["status"], "not_run")
        self.assertEqual(self.fake.mem[40], 0)
        self.assertIn("出力状态未知", report["notice"])
        jog.assert_not_called()

    def test_plan_expiry_during_motion_stops_new_targets_but_allows_cleanup(self):
        plan = self.prepare()
        self.now = 1299.95
        with mock.patch.object(servo, "jog_test") as jog:
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertIn("过期", report["error"])
        jog.assert_not_called()

    def test_keyboard_interrupt_in_motion_no_retry_but_cleanup(self):
        plan = self.prepare()
        original = self.fake.write2ByteTxRx
        count = 0
        def interrupt(sid, address, value):
            nonlocal count
            count += 1
            if count == 2:
                raise KeyboardInterrupt()
            return original(sid, address, value)
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=interrupt):
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(count, 2)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(report["jog50"]["status"], "not_run")

    def test_mid_run_node_change_stops_and_only_attempts_cleanup(self):
        plan = self.prepare()
        original = self.fake.write2ByteTxRx
        def unplug(sid, address, value):
            result = original(sid, address, value)
            self.node["ino"] += 1
            return result
        with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=unplug):
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(len([c for c in self.writes() if c[0] == "write2"]), 1)
        self.assertEqual(self.fake.mem[40], 0)
        self.assertEqual(report["jog50"]["status"], "not_run")

    def test_byte_level_loss_of_first_ack_no_cleanup_or_jog_packet(self):
        from servo_toolkit.strict_sdk import StrictHls
        class FirstLostAckPort(MotionMemoryPort):
            def writePort(port, data):
                count = super().writePort(data)
                if bytes(data)[4:6] == bytes([3, 42]):
                    port.reply.clear()  # first target executed; its ACK is lost
                return count
        port = FirstLostAckPort(self.fake)
        self.bus = servo.HlsBus(StrictHls(port), 0, -6)
        plan = self.prepare()
        with mock.patch.object(servo, "jog_test") as jog:
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertTrue(port._microduck_tainted)
        self.assertEqual(len([p for p in port.packets if p[4] == 3]), 1)
        self.assertIn("外部电源", report["error"])
        jog.assert_not_called()

    def test_final_position_must_still_be_in_center_window(self):
        self.position(2048)
        plan = self.prepare()
        real = servo.jog_test
        def drift(*args, **kwargs):
            result = real(*args, **kwargs)
            self.position(2058)
            return result
        with mock.patch.object(servo, "jog_test", side_effect=drift):
            code, report = self.cli_run(plan)
        self.assertEqual(code, 1)
        self.assertEqual(report["jog50"]["status"], "passed")
        self.assertEqual(report["final_inspect"]["status"], "failed_or_uncertain")
        self.assertIn("不自动回中", report["error"])

    def test_fake_permit_cannot_open_module_gate(self):
        before = self.bus.inspect(1)
        self.fake.calls.clear()
        for function, kwargs in ((servo.move_center, {"confirmation": servo.CONFIRM_MOVE_CENTER}),
                                 (servo.jog_test, {"confirmation": servo.CONFIRM_JOG, "amplitude": 50})):
            with self.assertRaises(servo.ServoError):
                function(self.bus, port=self.port, expected=before, records=self.records,
                         _permit=object(), **kwargs)
        self.assertEqual(self.fake.calls, [])
        self.assertFalse(self.records.exists())

    def test_scope_stage_and_bus_cannot_be_reused_or_swapped(self):
        plan = self.prepare("center")
        data = json.loads(Path(plan["plan_record"]).read_text())
        permit = motion._MotionPermit(data, self.bus, deadline=self.now + 300)
        before = servo.Snapshot(**data["snapshot"])
        with self.assertRaises(servo.ServoError):
            permit.claim(self.bus, self.port, before, "center")
        with self.assertRaises(servo.ServoError):
            permit.claim(permit.bus, self.port, before, "jog50")
        permit.claim(permit.bus, self.port, before, "center")
        with self.assertRaises(servo.ServoError):
            permit.claim(permit.bus, self.port, before, "center")
        with self.assertRaises(servo.ServoError):
            permit.allow_jog(replace(before, position=2048))

    def test_prepare_and_run_cli_need_no_tty_and_return_record_paths(self):
        self.position(2048)
        with mock.patch("builtins.input", side_effect=AssertionError("no input")), \
             mock.patch("sys.stdin.isatty", side_effect=AssertionError("no tty")), \
             mock.patch("sys.stdout.isatty", side_effect=AssertionError("no tty")):
            plan = self.prepare()
            code, report = self.cli_run(plan)
        self.assertEqual(code, 0)
        self.assertTrue(Path(report["jog50"]["result"]["record"]).is_file())
        self.assertTrue(Path(report["plan_record"]).is_file())

    def test_write_gate_still_blocks_controlled_commands(self):
        with mock.patch.object(cli, "WRITES_ENABLED", False), contextlib.redirect_stderr(io.StringIO()):
            code = cli.main(["--port", self.port, "motion-prepare", "--id", "1", "--scope", "center"])
        self.assertEqual(code, 2)
        self.connect.assert_not_called()
        self.assertFalse(self.records.exists())

    def test_fsync_stall_expiry_is_checked_again_before_connect(self):
        plan = self.prepare()
        original = motion._save_new
        def stall(*args, **kwargs):
            original(*args, **kwargs)
            self.now += 301
        with mock.patch.object(motion, "_save_new", side_effect=stall), \
             self.assertRaisesRegex(servo.ServoError, "未连接"):
            self.execute(plan)
        self.connect.assert_not_called()
        self.assertEqual(self.fake.calls, [])

    def test_each_permit_stage_can_only_be_claimed_once(self):
        self.position(2048)
        plan = self.prepare()
        data = json.loads(Path(plan["plan_record"]).read_text())
        permit = motion._MotionPermit(data, self.bus, deadline=self.now + 300)
        before = servo.Snapshot(**data["snapshot"])
        permit.claim(permit.bus, self.port, before, "center")
        permit.allow_jog(before)
        permit.claim(permit.bus, self.port, before, "jog50")
        with self.assertRaises(servo.ServoError):
            permit.allow_jog(before)

    def test_controlled_jog_pre_sync_fsync_drift_rejects_before_first_target(self):
        for start, drift in ((2048, 10), (2051, 3), (2045, -3), (2045, 6)):
            with self.subTest(start=start, drift=drift):
                self.position(start)
                plan = self.prepare()
                original = servo._record_result
                def change_after_fsync(path, event, data):
                    original(path, event, data)
                    if event == "pre_torque_guard" and "-jog-" in path.name:
                        self.position(start + drift)
                with mock.patch.object(servo, "_record_result", side_effect=change_after_fsync):
                    code, report = self.cli_run(plan)
                self.assertEqual(code, 1)
                self.assertEqual(report["jog50"]["status"], "failed_or_uncertain")
                self.assertEqual(self.writes(), [])
                self.assertEqual(self.fake.mem[40], 0)

    def test_controlled_jog_post_sync_drift_only_releases_no_jog_or_enable(self):
        for auto in (False, True):
            for start, drift in ((2048, 10), (2051, 3), (2045, -3), (2045, 6)):
                with self.subTest(automatic_enable=auto, start=start, drift=drift):
                    self.position(start)
                    self.fake.goal_enables_torque = auto
                    plan = self.prepare()
                    original = self.fake.write2ByteTxRx
                    def change_after_sync(sid, address, value):
                        result = original(sid, address, value)
                        self.position(start + drift)
                        return result
                    with mock.patch.object(self.fake, "write2ByteTxRx", side_effect=change_after_sync):
                        code, report = self.cli_run(plan)
                    self.assertEqual(code, 1)
                    self.assertEqual(report["jog50"]["status"], "failed_or_uncertain")
                    self.assertEqual(self.writes(), [("write2", 1, 42), ("write", 1, 40)])
                    self.assertEqual(self.fake.mem[40], 0)

    def test_controlled_jog_full_center_preconditions_survive_fsync(self):
        self.position(2048)
        for address, value in ((1, 45), (18, 0), (30, 2), (33, 0), (55, 0)):
            with self.subTest(address=address):
                plan = self.prepare()
                original = servo._record_result
                old = self.fake.mem[address]
                def change_config(path, event, data):
                    original(path, event, data)
                    if event == "pre_torque_guard" and "-jog-" in path.name:
                        self.fake.mem[address] = value
                with mock.patch.object(servo, "_record_result", side_effect=change_config):
                    code, report = self.cli_run(plan)
                self.fake.mem[address] = old
                self.assertEqual(code, 1)
                self.assertEqual(self.writes(), [])

    def test_complete_dialogue_id_baud_motion_and_power_cycle_readonly(self):
        # End-to-end CLI simulation, including ID 1->12 and explicit mock power reset.
        # This is not physical power, UART or real EEPROM persistence evidence.
        output, error = io.StringIO(), io.StringIO()
        with mock.patch.object(cli, "RECORDS", self.records), \
             mock.patch.object(cli, "connect", side_effect=self.connect.side_effect), \
             contextlib.redirect_stdout(output), contextlib.redirect_stderr(error), \
             mock.patch("builtins.input", side_effect=AssertionError("no input")), \
             mock.patch("sys.stdin.isatty", side_effect=AssertionError("no tty")), \
             mock.patch("sys.stdout.isatty", side_effect=AssertionError("no tty")):
            base = ["--port", self.port, "--baud", "1000000"]
            self.assertEqual(cli.main([*base, "inspect", "--id", "1"]), 0)
            self.assertEqual(cli.main([*base, "set-id", "--id", "1", "--new-id", "12"]), 0)
            self.assertEqual(self.fake.sid, 12)
            self.assertEqual(self.fake.mem[6], 0)
            output.seek(0); output.truncate()
            self.assertEqual(cli.main([*base, "motion-prepare", "--id", "12",
                                       "--scope", "center-jog50"]), 0)
            plan = json.loads(output.getvalue())
            output.seek(0); output.truncate()
            self.assertEqual(cli.main([*base, "motion-run", "--id", "12", "--scope", "center-jog50",
                                       "--plan", plan["plan_id"], "--approve", plan["approve_sha256"]]), 0)
            report = json.loads(output.getvalue())
            self.assertEqual(report["center"]["status"], "passed")
            self.assertEqual(report["jog50"]["status"], "passed")
            self.assertEqual(report["final_inspect"]["snapshot"]["position"], 2048)
            self.node["ino"] += 1  # explicit mock disconnect/re-enumeration
            self.fake.mem[40] = self.fake.mem[42] = self.fake.mem[43] = 0
            self.fake.calls.clear()
            self.assertEqual(cli.main([*base, "inspect", "--id", "12"]), 0)
            self.assertEqual(cli.main([*base, "inspect", "--id", "1"]), 1)
            self.assertIn("通信结果=-6，舵机状态=0x00", error.getvalue())
            self.assertEqual(self.writes(), [])  # reconnect verification never compensates
            output.seek(0); output.truncate()
            self.assertEqual(cli.main([*base, "motion-run", "--id", "12", "--scope", "center-jog50",
                                       "--plan", plan["plan_id"], "--approve", plan["approve_sha256"]]), 1)
            self.assertEqual(self.writes(), [])
        self.assertFalse(servo.MOVE_CENTER_ARMED or servo.JOG_ARMED)
        self.assertFalse(cli.MOVE_CENTER_ENABLED or cli.JOG_ENABLED)

    def test_blocking_load_consume_connect_never_renews_monotonic_lifetime(self):
        scenarios = (("connect", 301, 999, 0), ("connect", 301, 1001, 0),
                     ("connect", 301, 1301, 0), ("connect", 10, 1001, 295),
                     ("connect", 1, 999, 0), ("consume", 301, 1001, 0),
                     ("load", 301, 1001, 0))
        for boundary, delay, after_wall, prior_wait in scenarios:
            with self.subTest(boundary=boundary, delay=delay, wall=after_wall, waited=prior_wait):
                clock = {"wall": 1000.0, "mono": 500.0}
                def sleep(seconds):
                    clock["wall"] += seconds
                    clock["mono"] += seconds
                def stall():
                    clock["mono"] += delay
                    clock["wall"] = after_wall
                @contextlib.contextmanager
                def connect(*args, **kwargs):
                    if boundary == "connect":
                        stall()
                    yield self.bus
                with mock.patch.object(motion.time, "time", side_effect=lambda: clock["wall"]), \
                     mock.patch.object(motion.time, "monotonic", side_effect=lambda: clock["mono"]), \
                     mock.patch.object(motion.time, "sleep", side_effect=sleep):
                    self.position(2048)
                    plan = self.prepare()
                    sleep(prior_wait)
                    real_save, real_load = motion._save_new, motion._load
                    def save(*args, **kwargs):
                        result = real_save(*args, **kwargs)
                        if boundary == "consume":
                            stall()
                        return result
                    def load(*args, **kwargs):
                        result = real_load(*args, **kwargs)
                        if boundary == "load":
                            stall()
                        return result
                    with mock.patch.object(servo, "connect", side_effect=connect) as opening, \
                         mock.patch.object(motion, "_save_new", side_effect=save), \
                         mock.patch.object(motion, "_load", side_effect=load):
                        code, report = self.cli_run(plan)
                    self.assertEqual(code, 1)
                    self.assertIn("过期", report["error"])
                    self.assertEqual(report["center"]["status"], "not_run")
                    self.assertEqual(report["jog50"]["status"], "not_run")
                    self.assertEqual(self.writes(), [])
                    self.assertEqual(self.fake.calls, [])
                    self.assertEqual(opening.call_count, 1 if boundary == "connect" else 0)
                    self.assertTrue((self.records / ".motion-plans" / (plan["plan_id"] + ".used")).is_file())

    def test_short_blocking_connect_keeps_remaining_lifetime_and_passes(self):
        clock = {"wall": 1000.0, "mono": 500.0}
        def sleep(seconds):
            clock["wall"] += seconds
            clock["mono"] += seconds
        @contextlib.contextmanager
        def connect(*args, **kwargs):
            sleep(1)
            yield self.bus
        with mock.patch.object(motion.time, "time", side_effect=lambda: clock["wall"]), \
             mock.patch.object(motion.time, "monotonic", side_effect=lambda: clock["mono"]), \
             mock.patch.object(motion.time, "sleep", side_effect=sleep):
            self.position(2048)
            plan = self.prepare()
            with mock.patch.object(servo, "connect", side_effect=connect):
                code, report = self.cli_run(plan)
        self.assertEqual(code, 0)
        self.assertEqual(report["jog50"]["status"], "passed")
        self.assertEqual(self.fake.mem[40], 0)
