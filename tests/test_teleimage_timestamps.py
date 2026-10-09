import unittest

import numpy as np

from teleimager.image_client import SimpleFPSMonitor, TripleRingBuffer, ZMQ_SubscriberThread


class TeleImageTimestampTest(unittest.TestCase):
    @staticmethod
    def _subscriber(*, request_bgr):
        # Build only the read-side state; no socket or background thread is
        # needed to verify that a frame and its receive timestamps stay paired.
        subscriber = object.__new__(ZMQ_SubscriberThread)
        subscriber._request_bgr = request_bgr
        subscriber._fps_monitor = SimpleFPSMonitor(window_size=1)
        subscriber._jpg_3ring_buffer = TripleRingBuffer()
        subscriber._bgr_3ring_buffer = TripleRingBuffer() if request_bgr else None
        return subscriber

    def test_jpeg_receive_timestamp_stays_with_jpeg(self):
        subscriber = self._subscriber(request_bgr=False)
        subscriber._jpg_3ring_buffer.write((b"jpeg", 101, 202))

        image = subscriber.recv()

        self.assertEqual(image.jpg, b"jpeg")
        self.assertEqual(image.workstation_receive_monotonic_ns, 101)
        self.assertEqual(image.workstation_receive_realtime_ns, 202)

    def test_bgr_recorder_uses_timestamp_of_matching_decoded_frame(self):
        subscriber = self._subscriber(request_bgr=True)
        subscriber._jpg_3ring_buffer.write((b"newer-jpeg", 301, 302))
        bgr = np.zeros((2, 3, 3), dtype=np.uint8)
        subscriber._bgr_3ring_buffer.write((bgr, 201, 202))

        image = subscriber.recv()

        self.assertIs(image.bgr, bgr)
        self.assertEqual(image.workstation_receive_monotonic_ns, 201)
        self.assertEqual(image.workstation_receive_realtime_ns, 202)


if __name__ == "__main__":
    unittest.main()
