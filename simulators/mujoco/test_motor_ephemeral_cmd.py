"""Unit tests for ephemeral motor command gating (MuJoCo controller)."""

import unittest

from motor_ephemeral_utils import motor_rx_is_new_packet as _motor_rx_is_new_packet


class TestMotorRxIsNewPacket(unittest.TestCase):
    """Guards FEAGI motor ephemeral policy: new packet iff rx sequence advances."""

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


if __name__ == "__main__":
    unittest.main()
