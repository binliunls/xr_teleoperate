import json
import os
import tempfile
import unittest


try:
    from teleop.utils.episode_writer import EpisodeWriter
except ModuleNotFoundError as exc:  # Minimal developer shells may omit cv2/rerun/logging_mp.
    EpisodeWriter = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


@unittest.skipIf(EpisodeWriter is None, f"EpisodeWriter dependencies unavailable: {_IMPORT_ERROR}")
class EpisodeWriterCaptureFieldsTest(unittest.TestCase):
    def _record_one(self, root, *, timestamps=None, native_capture=None, actions=None):
        writer = EpisodeWriter(root, rerun_log=False)
        self.assertTrue(writer.create_episode())
        idx = writer.add_item(
            colors={},
            states={"left_arm": {}},
            actions={"left_arm": {}} if actions is None else actions,
            timestamps=timestamps,
            native_capture=native_capture,
        )
        episode_dir = writer.episode_dir
        writer.save_episode()
        writer.close()
        with open(os.path.join(episode_dir, "data.json"), "r", encoding="utf-8") as handle:
            return idx, json.load(handle)["data"][0]

    def test_additive_capture_fields_use_exact_writer_idx(self):
        with tempfile.TemporaryDirectory() as root:
            idx, item = self._record_one(
                root,
                timestamps={
                    "workstation_monotonic_ns": 100,
                    "workstation_realtime_ns": 200,
                    "cameras": {},
                },
                native_capture={
                    "protocol": "sharpa.capture.v1",
                    "capture_id": "capture-1",
                    "workstation_monotonic_ns": 100,
                    "workstation_realtime_ns": 200,
                },
            )
        self.assertEqual(idx, 0)
        self.assertEqual(item["idx"], idx)
        self.assertEqual(item["native_capture"]["idx"], idx)
        self.assertEqual(item["native_capture"]["capture_id"], "capture-1")
        self.assertEqual(item["timestamps"]["workstation_monotonic_ns"], 100)
        for legacy_key in (
            "idx",
            "colors",
            "depths",
            "states",
            "actions",
            "tactiles",
            "audios",
            "sim_state",
        ):
            self.assertIn(legacy_key, item)

    def test_legacy_caller_gets_legacy_item_schema(self):
        with tempfile.TemporaryDirectory() as root:
            _, item = self._record_one(root)
        self.assertNotIn("timestamps", item)
        self.assertNotIn("native_capture", item)

    def test_sharpa_desired_command_is_additive_to_legacy_action(self):
        with tempfile.TemporaryDirectory() as root:
            _, item = self._record_one(
                root,
                actions={
                    "left_ee": {
                        "qpos": [0.1],
                        "desired_qpos": [0.2],
                    }
                },
                timestamps={
                    "sharpa_hand_desired_command": {
                        "left": {"workstation_receive_monotonic_ns": 123}
                    }
                },
            )
        self.assertEqual(item["actions"]["left_ee"]["qpos"], [0.1])
        self.assertEqual(item["actions"]["left_ee"]["desired_qpos"], [0.2])
        self.assertEqual(
            item["timestamps"]["sharpa_hand_desired_command"]["left"][
                "workstation_receive_monotonic_ns"
            ],
            123,
        )


if __name__ == "__main__":
    unittest.main()
