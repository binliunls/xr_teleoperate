import threading
import unittest

import numpy as np


try:
    from teleop.robot_control.robot_arm import (
        DataBuffer,
        H2_ArmController,
        H2_JointArmIndex,
        H2_LowState,
        _h2_lowstate_timestamps,
    )
    from teleop.robot_control.robot_hand_sharpa import (
        SHARPA_DOF,
        SharpaTactile_Subscriber,
        SharpaWave_Controller,
        _sharpa_desired_q,
    )
    from teleop.utils.sharpa_tactile_wire import TactileMessage
except ModuleNotFoundError as exc:
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


@unittest.skipIf(_IMPORT_ERROR is not None, f"teleop dependencies unavailable: {_IMPORT_ERROR}")
class RecordingSignalTimestampTest(unittest.TestCase):
    def test_h2_preserves_tick_sequence_and_receive_clocks(self):
        msg = type("Message", (), {"tick": 1234, "sequence": 77})()

        timestamps = _h2_lowstate_timestamps(msg, 100, 200)

        self.assertEqual(
            timestamps,
            {
                "workstation_receive_monotonic_ns": 100,
                "workstation_receive_realtime_ns": 200,
                "unitree_tick": 1234,
                "source_sequence": 77,
            },
        )

    def test_h2_recording_snapshot_pairs_values_and_timing(self):
        lowstate = H2_LowState()
        for index, motor in enumerate(lowstate.motor_state):
            motor.q = float(index)
            motor.dq = float(-index)
        lowstate.timestamps = {"unitree_tick": 9, "workstation_receive_monotonic_ns": 10}
        buffer = DataBuffer()
        buffer.SetData(lowstate)
        controller = object.__new__(H2_ArmController)
        controller.lowstate_buffer = buffer

        snapshot = controller.get_recording_state_snapshot()

        expected_arm_q = [float(index.value) for index in H2_JointArmIndex]
        self.assertEqual(snapshot["dual_arm_q"].tolist(), expected_arm_q)
        self.assertEqual(snapshot["timestamps"], lowstate.timestamps)
        self.assertEqual(snapshot["waist_q"].tolist(), [12.0, 13.0, 14.0])

    def test_h2_command_tracks_target_set_and_successful_publish(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def Write(self, msg):
                self.messages.append(msg)
                return True

        controller = object.__new__(H2_ArmController)
        controller.ctrl_lock = threading.Lock()
        controller.q_target = np.zeros(14)
        controller.tauff_target = np.zeros(14)
        controller._command_target_generation = 0
        controller._command_target_timestamps = {}
        controller._last_command_publish_timestamps = {}
        controller.lowcmd_publisher = Publisher()

        controller.ctrl_dual_arm(np.ones(14), np.zeros(14))
        target = controller.get_command_timing_snapshot()["target"]
        published = controller._write_lowcmd_with_timing(
            object(),
            target_generation=target["target_generation"],
            target_timestamps=target,
        )
        timestamps = controller.get_command_timing_snapshot()

        self.assertTrue(published)
        self.assertEqual(timestamps["target"]["target_generation"], 1)
        self.assertEqual(timestamps["last_publish"]["target_generation"], 1)
        self.assertIn("workstation_publish_monotonic_ns", timestamps["last_publish"])
        self.assertIn("workstation_publish_realtime_ns", timestamps["last_publish"])
        self.assertEqual(len(controller.lowcmd_publisher.messages), 1)

    def test_h2_failed_publish_preserves_last_successful_timing(self):
        class Publisher:
            def __init__(self):
                self.succeed = True

            def Write(self, _msg):
                return self.succeed

        controller = object.__new__(H2_ArmController)
        controller.ctrl_lock = threading.Lock()
        controller._last_command_publish_timestamps = {}
        controller.lowcmd_publisher = Publisher()

        self.assertTrue(
            controller._write_lowcmd_with_timing(
                object(),
                target_generation=1,
                target_timestamps={"target_generation": 1},
            )
        )
        successful_timing = dict(controller._last_command_publish_timestamps)

        controller.lowcmd_publisher.succeed = False
        self.assertFalse(
            controller._write_lowcmd_with_timing(
                object(),
                target_generation=2,
                target_timestamps={"target_generation": 2},
            )
        )
        self.assertEqual(controller._last_command_publish_timestamps, successful_timing)

    def test_sharpa_state_snapshot_copies_receive_metadata(self):
        controller = object.__new__(SharpaWave_Controller)
        controller._snapshot_lock = threading.Lock()
        controller._left_angles = np.arange(SHARPA_DOF, dtype=np.float64)
        controller._right_angles = np.arange(SHARPA_DOF, dtype=np.float64) + 100
        controller._state_timestamps = {
            "left": {"workstation_receive_monotonic_ns": 11},
            "right": {"workstation_receive_monotonic_ns": 22},
        }
        controller._desired_q = {
            "left": np.arange(SHARPA_DOF, dtype=np.float64) / 10,
            "right": None,
        }
        controller._desired_timestamps = {
            "left": {"workstation_receive_monotonic_ns": 33},
            "right": {},
        }

        snapshot = controller.get_state_snapshot()
        controller._left_angles[0] = -1
        controller._desired_q["left"][0] = -1

        self.assertEqual(snapshot["left"][0], 0.0)
        self.assertEqual(
            snapshot["timestamps"]["right"]["workstation_receive_monotonic_ns"],
            22,
        )
        self.assertEqual(snapshot["desired_q"]["left"][0], 0.0)
        self.assertIsNone(snapshot["desired_q"]["right"])
        self.assertEqual(
            snapshot["desired_timestamps"]["left"]["workstation_receive_monotonic_ns"],
            33,
        )

    def test_sharpa_desired_command_requires_complete_vector(self):
        motor = lambda q: type("MotorCommand", (), {"q": q})()
        complete = type(
            "HandCommand",
            (),
            {"motor_cmd": [motor(index / 10) for index in range(SHARPA_DOF)]},
        )()
        partial = type("HandCommand", (), {"motor_cmd": [motor(0.0)]})()

        self.assertEqual(_sharpa_desired_q(complete).shape, (SHARPA_DOF,))
        self.assertIsNone(_sharpa_desired_q(partial))
        self.assertIsNone(_sharpa_desired_q(None))

    def test_tactile_frame_keeps_source_and_receive_times(self):
        subscriber = object.__new__(SharpaTactile_Subscriber)
        subscriber._lock = threading.Lock()
        subscriber._cache = {}
        message = TactileMessage(
            channel=0,
            frame_id=12,
            ts=3.5,
            deform=np.zeros((2, 2), dtype=np.uint8),
            f6=np.arange(6, dtype=np.float32),
            contact_point=np.zeros(0, dtype=np.float32),
        )

        subscriber._store_message(
            message,
            receive_monotonic_ns=101,
            receive_realtime_ns=202,
        )
        finger = subscriber.snapshot()["right_ee"]["pinky"]

        self.assertEqual(finger["frame_id"], 12)
        self.assertEqual(finger["ts"], 3.5)
        self.assertEqual(finger["workstation_receive_monotonic_ns"], 101)
        self.assertEqual(finger["workstation_receive_realtime_ns"], 202)
        self.assertNotIn("joint_qpos", finger)

    def test_tactile_frame_converts_joint_trailer_to_radian_qpos(self):
        subscriber = object.__new__(SharpaTactile_Subscriber)
        subscriber._lock = threading.Lock()
        subscriber._cache = {}
        joints_deg = np.arange(SHARPA_DOF, dtype=np.float32) * 10.0
        message = TactileMessage(
            channel=5,
            frame_id=13,
            ts=4.5,
            deform=np.zeros((2, 2), dtype=np.uint8),
            f6=np.arange(6, dtype=np.float32),
            contact_point=np.zeros(0, dtype=np.float32),
            joints=joints_deg,
        )

        subscriber._store_message(
            message,
            receive_monotonic_ns=303,
            receive_realtime_ns=404,
        )
        finger = subscriber.snapshot()["left_ee"]["pinky"]

        self.assertEqual(len(finger["joint_qpos"]), SHARPA_DOF)
        np.testing.assert_allclose(
            finger["joint_qpos"],
            np.deg2rad(joints_deg.astype(np.float64)),
            rtol=0.0,
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
