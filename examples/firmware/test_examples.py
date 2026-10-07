import unittest
from attitude_to_python import orientation
from aerial_kit.dynamics.rotations import quat_to_rotmat
import numpy as np
import read_board

class ExamplesTest(unittest.TestCase):
    def test_north_becomes_positive_enu_y(self):
        q = orientation(dict(roll_deg=0, pitch_deg=0, yaw_deg=0))
        np.testing.assert_allclose(quat_to_rotmat(q) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(quat_to_rotmat(q) @ [0, 0, 1], [0, 0, 1], atol=1e-12)

    def test_east_becomes_positive_enu_x(self):
        q = orientation(dict(roll_deg=0, pitch_deg=0, yaw_deg=90))
        np.testing.assert_allclose(quat_to_rotmat(q), np.eye(3), atol=1e-12)

    def test_nonfinite_attitude_is_rejected(self):
        with self.assertRaises(ValueError):
            orientation(dict(roll_deg=float('nan'), pitch_deg=0, yaw_deg=0))

    def test_read_only_status_request_and_units(self):
        class Client:
            def request(self, command):
                self.command = command
                # fixed captured-shape STATUS payload: 12.3/-4.5/90 deg
                return bytes.fromhex('000001067b00d3ff8403000000000000000000000000')
        client = Client()
        sample = read_board.read_once(client)
        self.assertEqual(client.command, read_board.akproto.STATUS)
        self.assertEqual((sample['roll_deg'], sample['pitch_deg'], sample['yaw_deg']), (12.3, -4.5, 90.0))

if __name__ == '__main__': unittest.main()
