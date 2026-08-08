"""Unit tests for FEAGI motor rx / hold-pose helpers (MuJoCo controller)."""

import unittest

from motor_ephemeral_utils import (
    motor_rx_is_new_packet as _motor_rx_is_new_packet,
    resolve_servo_ctrl_command,
)


class TestMotorRxIsNewPacket(unittest.TestCase):
    """New motor packet iff rx sequence advances (used for sparse-command hold logic)."""

    def test_advances_from_initial(self):
        self.assertTrue(_motor_rx_is_new_packet(0, -1))

    def test_advances_when_seq_increments(self):
        self.assertTrue(_motor_rx_is_new_packet(1, 0))

    def test_not_new_when_seq_unchanged(self):
        self.assertFalse(_motor_rx_is_new_packet(1, 1))

    def test_not_new_when_seq_missing(self):
        self.assertFalse(_motor_rx_is_new_packet(None, 0))

    def test_not_new_when_seq_not_int(self):
        self.assertFalse(_motor_rx_is_new_packet("1", 0))


class TestResolveServoCtrlCommand(unittest.TestCase):
    """Absolute one-shot: command when snap present, hold pose when absent."""

    def test_maps_absolute_mid_to_center(self):
        ctrl, mode = resolve_servo_ctrl_command(
            snap_value=0.5,
            current_qpos=1.0,
            min_val=-3.14159,
            max_val=3.14159,
        )
        self.assertEqual(mode, "command")
        self.assertAlmostEqual(ctrl, 0.0, places=5)

    def test_maps_absolute_max_to_range_max(self):
        ctrl, mode = resolve_servo_ctrl_command(
            snap_value=1.0,
            current_qpos=0.0,
            min_val=-3.14159,
            max_val=3.14159,
        )
        self.assertEqual(mode, "command")
        self.assertAlmostEqual(ctrl, 3.14159, places=5)

    def test_hold_pose_uses_current_qpos(self):
        ctrl, mode = resolve_servo_ctrl_command(
            snap_value=None,
            current_qpos=0.42,
            min_val=-1.0,
            max_val=1.0,
        )
        self.assertEqual(mode, "hold_pose")
        self.assertAlmostEqual(ctrl, 0.42, places=5)

    def test_hold_pose_clamps_qpos_to_range(self):
        ctrl, mode = resolve_servo_ctrl_command(
            snap_value=None,
            current_qpos=5.0,
            min_val=-1.0,
            max_val=1.0,
        )
        self.assertEqual(mode, "hold_pose")
        self.assertAlmostEqual(ctrl, 1.0, places=5)

    def test_hold_pose_falls_back_to_center_without_qpos(self):
        ctrl, mode = resolve_servo_ctrl_command(
            snap_value=None,
            current_qpos=None,
            min_val=-2.0,
            max_val=4.0,
        )
        self.assertEqual(mode, "hold_pose")
        self.assertAlmostEqual(ctrl, 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
