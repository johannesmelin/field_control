import unittest
from pathlib import Path

from field_control.motor_boundary import (
    LEFT_ID, RIGHT_ID, OBSERVED_RMDX_SAME_ID_REPLY_PROFILE,
    get_motor_reply_profile, v38_speed_frame,
)


class MotorBoundaryProtocolTests(unittest.TestCase):
    def test_v38_speed_frame_is_signed_001_degree_per_second(self):
        self.assertEqual(v38_speed_frame(LEFT_ID, -6.0),
                         bytes((0xA2, 0, 0, 0, 0xA8, 0xFD, 0xFF, 0xFF)))

    def test_installed_same_id_profile_must_be_named_explicitly(self):
        self.assertIs(get_motor_reply_profile("observed-rmdx-same-id"), OBSERVED_RMDX_SAME_ID_REPLY_PROFILE)
        with self.assertRaises(ValueError):
            get_motor_reply_profile(None)

    def test_legacy_operational_transport_names_are_absent(self):
        import field_control.motor_boundary as boundary
        self.assertFalse(hasattr(boundary, "SocketCanV38Transport"))
        self.assertFalse(hasattr(boundary, "PhysicalMotorBoundary"))

    def test_ambiguous_legacy_rpm_cap_name_is_absent_from_sources(self):
        legacy = "VERIFIED_MAX_" + "WHEEL_RPM"
        import field_control.motor_boundary as boundary
        self.assertFalse(hasattr(boundary, legacy))
        root = Path(__file__).resolve().parents[1]
        for source in root.rglob("*.py"):
            self.assertNotIn(legacy, source.read_text(), source)

    def test_protocol_encoder_rejects_unknown_motor_id(self):
        with self.assertRaises(ValueError):
            v38_speed_frame(RIGHT_ID + 1, 1.0)


if __name__ == "__main__":
    unittest.main()
