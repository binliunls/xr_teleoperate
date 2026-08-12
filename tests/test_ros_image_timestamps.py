import unittest

import numpy as np

from teleop.utils.ros_image_client import _Subscriber


class ROSImageTimestampTest(unittest.TestCase):
    def test_subscriber_keeps_source_and_receive_times_with_frame(self):
        subscriber = _Subscriber()
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        subscriber.push(
            frame,
            source_timestamp_ns=123,
            workstation_receive_monotonic_ns=456,
            workstation_receive_realtime_ns=789,
        )
        saved_frame, _, timestamps = subscriber.snapshot()
        self.assertIs(saved_frame, frame)
        self.assertEqual(
            timestamps,
            {
                "ros_header_stamp_ns": 123,
                "workstation_receive_monotonic_ns": 456,
                "workstation_receive_realtime_ns": 789,
            },
        )


if __name__ == "__main__":
    unittest.main()
