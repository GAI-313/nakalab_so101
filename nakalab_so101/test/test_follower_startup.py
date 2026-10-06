"""疑似シリアルポートで C++ ドライバの実際の起動通信を検証する．"""

import argparse
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import tempfile
import threading
import time
import tty
import unittest


class ServoEmulator:
    """実機デバイスを開かず，6 個のサーボの通信を再現する．"""

    def __init__(self, *, ignore_goals=False, outside_limits=False,
                 wrong_id=False, servo_error=0, wrong_size=False,
                 torque_enabled=False, above_limits=False):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.device = os.ttyname(self.slave)
        self.ignore_goals = ignore_goals
        self.wrong_id = wrong_id
        self.servo_error = servo_error
        self.wrong_size = wrong_size
        self.requests = []
        self.registers = {}
        for motor_id in range(1, 7):
            registers = bytearray(64)
            struct.pack_into('<H', registers, 9, 1000)
            struct.pack_into('<H', registers, 11, 3000)
            struct.pack_into('<H', registers, 42, 1200)
            registers[40] = int(torque_enabled)
            position = 900 if outside_limits and motor_id == 1 else 2048
            if above_limits and motor_id == 5:
                struct.pack_into('<H', registers, 9, 140)
                struct.pack_into('<H', registers, 11, 3968)
                position = 3993
            struct.pack_into('<H', registers, 56, position)
            self.registers[motor_id] = registers
        self.stop = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self._serve)
        self.thread.start()

    def _serve(self):
        buffer = bytearray()
        try:
            while not self.stop.is_set():
                readable, _, _ = select.select([self.master], [], [], 0.05)
                if not readable:
                    continue
                buffer.extend(os.read(self.master, 4096))
                while len(buffer) >= 4:
                    if buffer[:2] != b'\xff\xff':
                        raise AssertionError('送信パケットのヘッダが不正です．')
                    size = buffer[3] + 4
                    if len(buffer) < size:
                        break
                    packet = bytes(buffer[:size])
                    del buffer[:size]
                    if sum(packet[2:]) & 0xff != 0xff:
                        raise AssertionError('送信チェックサムが不正です．')
                    self._handle(packet[2], packet[4], packet[5:-1])
        except Exception as error:
            self.failure = error

    def _handle(self, motor_id, instruction, parameters):
        self.requests.append((motor_id, instruction, parameters))
        if instruction == 2:
            address, size = parameters
            payload = self.registers[motor_id][address:address + size]
            if self.wrong_size:
                payload += b'\x00'
            reply_id = motor_id % 6 + 1 if self.wrong_id else motor_id
            self._reply(reply_id, self.servo_error, payload)
        elif instruction == 3:
            self._write(motor_id, parameters[0], parameters[1:])
            self._reply(motor_id, 0, b'')
        elif instruction == 0x83:
            address, size = parameters[:2]
            for index in range(2, len(parameters), size + 1):
                self._write(parameters[index], address,
                            parameters[index + 1:index + 1 + size])
        else:
            raise AssertionError(f'想定外の命令: {instruction}')

    def _write(self, motor_id, address, data):
        if self.ignore_goals and address == 42:
            return
        self.registers[motor_id][address:address + len(data)] = data

    def _reply(self, motor_id, error, payload):
        body = bytes([motor_id, len(payload) + 2, error]) + payload
        os.write(self.master, b'\xff\xff' + body + bytes([~sum(body) & 0xff]))

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)
        if self.thread.is_alive():
            raise AssertionError('模擬サーボのスレッドが終了しません．')
        if self.failure is not None:
            raise self.failure


class FollowerStartupTest(unittest.TestCase):
    """実行ファイルの起動から終了までの書き込みを検査する．"""

    driver = None

    def run_driver(self, emulator, *, active=False, expect_exit=False):
        with tempfile.TemporaryDirectory(prefix='so101_startup_') as directory:
            environment = dict(os.environ)
            environment.update(
                ROS_DOMAIN_ID='42', ROS_LOCALHOST_ONLY='1',
                RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=directory,
            )
            command = [
                str(self.driver), '--ros-args',
                '-p', f'device:={emulator.device}',
                '-p', f'calibration_root:={directory}',
                '-r', f'__ns:=/codex_so101_test_{os.getpid()}',
            ]
            # 起動ガードの検証には，校正ファイルも模擬値で作成する．
            calibration = Path(directory) / 'arm.json'
            names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                     'wrist_flex', 'wrist_roll', 'gripper']
            calibration.write_text(json.dumps({
                name: {'homing_offset': 0} for name in names
            }), encoding='utf-8')
            command.extend(['-p', f'calibration_file:={calibration}'])
            if active:
                command.extend(['-p', 'read_only:=false'])
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                env=environment, text=True,
            )
            print(f'模擬ドライバ PID={process.pid} 停止=kill -INT {process.pid}',
                  flush=True)
            try:
                deadline = time.monotonic() + 4
                while process.poll() is None and time.monotonic() < deadline:
                    feedback_reads = sum(
                        instruction == 2 and parameters == bytes([56, 6])
                        for _, instruction, parameters in emulator.requests
                    )
                    required_reads = 24 if active else 12
                    if not expect_exit and feedback_reads >= required_reads:
                        break
                    time.sleep(0.01)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    output, _ = process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    output, _ = process.communicate(timeout=3)
                print(f'模擬ドライバ終了 PID={process.pid} rc={process.returncode}',
                      flush=True)
            return process.returncode, output

    def check_rejected(self, expected_message=None, **fault):
        emulator = ServoEmulator(**fault)
        try:
            status, output = self.run_driver(emulator, active=True, expect_exit=True)
            enabled = [request for request in emulator.requests
                       if request[1] == 3 and request[2] == bytes([40, 1])]
            self.assertEqual(enabled, [], output)
            self.assertEqual(status, 1, output)
            if expected_message is not None:
                self.assertIn(expected_message, output)
        finally:
            emulator.close()

    def test_default_mode_never_writes_even_on_shutdown(self):
        emulator = ServoEmulator()
        try:
            status, output = self.run_driver(emulator)
            writes = [request for request in emulator.requests
                      if request[1] != 2]
            self.assertEqual(writes, [], output)
            self.assertEqual(status, 0, output)
            self.assertIn('read_only', output)
        finally:
            emulator.close()

    def test_dropped_goal_write_does_not_enable_torque(self):
        self.check_rejected(ignore_goals=True)

    def test_outside_limits_does_not_enable_torque(self):
        self.check_rejected(
            outside_limits=True,
            expected_message='raw=900 limits=[1000,3000] 超過=100 ステップ',
        )

    def test_wrist_roll_above_limit_reports_exact_excess(self):
        self.check_rejected(
            above_limits=True,
            expected_message='wrist_roll id=5: 現在位置が可動範囲外です: '
            'raw=3993 limits=[140,3968] 超過=25 ステップ (2.197266 度)',
        )

    def test_wrong_motor_reply_does_not_enable_torque(self):
        self.check_rejected(wrong_id=True)

    def test_servo_error_does_not_enable_torque(self):
        self.check_rejected(servo_error=8, expected_message='error=8 (過電流)')

    def test_voltage_error_is_distinguished_from_invalid_response(self):
        self.check_rejected(servo_error=1, expected_message='error=1 (入力電圧異常)')

    def test_combined_servo_errors_are_reported(self):
        self.check_rejected(
            servo_error=37, expected_message='error=37 (入力電圧異常, 過熱, 過負荷)',
        )

    def test_wrong_reply_size_does_not_enable_torque(self):
        self.check_rejected(wrong_size=True)

    def test_enabled_torque_does_not_allow_startup(self):
        self.check_rejected(
            torque_enabled=True, expected_message='トルクが既に有効',
        )

    def test_control_mode_verifies_goals_before_enabling_torque(self):
        emulator = ServoEmulator()
        try:
            status, output = self.run_driver(emulator, active=True)
            self.assertEqual(status, 0, output)
            enabled = next(index for index, request in enumerate(emulator.requests)
                           if request[1:] == (3, bytes([40, 1])))
            goal_write = next(index for index, request in enumerate(emulator.requests)
                              if request[1] == 0x83)
            checked_ids = {
                motor_id for motor_id, instruction, parameters
                in emulator.requests[goal_write + 1:enabled]
                if instruction == 2 and parameters == bytes([42, 2])
            }
            self.assertEqual(checked_ids, set(range(1, 7)), output)
        finally:
            emulator.close()

    def test_negative_position_uses_sign_magnitude(self):
        emulator = ServoEmulator()
        struct.pack_into('<H', emulator.registers[1], 56, 0x8001)
        try:
            status, output = self.run_driver(emulator)
            self.assertEqual(status, 0, output)
            self.assertIn('shoulder_pan id=1 raw=-1', output)
            self.assertTrue(all(request[1] == 2 for request in emulator.requests))
        finally:
            emulator.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', type=Path, required=True)
    arguments, remaining = parser.parse_known_args()
    FollowerStartupTest.driver = arguments.driver.resolve()
    unittest.main(argv=['test_follower_startup', *remaining])
