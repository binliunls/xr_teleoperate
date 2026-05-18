"""
Wire format for the Sharpa tactile ZMQ stream.

Shared truth between the C++ bridge (scripts/sharpa_dds_bridge.cpp, PUB) and the
Python subscriber + sim stub (robot_control/robot_hand_sharpa.py SUB,
scripts/sharpa_tactile_sim.py PUB).

One message = one fingertip's frame. The bridge emits 10 messages per
hardware tick (5 fingers × 2 hands).

Layout (little-endian, 28-byte header followed by raw payload bytes):

    offset  type       field
    ------  ---------- --------------------------------
    0       uint32     channel        (0-9; 0-4=right, 5-9=left)
    4       uint32     frame_id       (SDK frame counter)
    8       float64    ts             (SDK hardware timestamp, seconds)
    16      uint32     deform_len     (== 57600 for 240x240 uint8)
    20      uint32     f6_len         (== 24 for 6 float32)
    24      uint32     cp_len         (variable, multiples of 12)
    28      uint8[]    deform bytes
    +dlen   float32[]  f6 values
    +flen   float32[]  contact_point triples (x, y, conf) per pixel

Match the matching C++ packing in scripts/sharpa_dds_bridge.cpp.
"""

import struct
from dataclasses import dataclass

import numpy as np


HEADER_FORMAT = "<IIdIII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 28

DEFORM_H = 240
DEFORM_W = 240
DEFORM_BYTES = DEFORM_H * DEFORM_W  # 57600
F6_LEN = 6
F6_BYTES = F6_LEN * 4  # 24

NUM_CHANNELS = 10
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def channel_to_hand_finger(channel):
    """Map SDK channel index (0-9) to ('left_ee'|'right_ee', finger_name).

    Sharpa SDK convention (sample/c++/tactile_sample_common.hpp:133):
    channels 0-4 belong to the right hand, 5-9 to the left hand. Within
    each hand, channel % 5 indexes thumb, index, middle, ring, pinky.
    """
    if not (0 <= channel < NUM_CHANNELS):
        raise ValueError(f"channel out of range [0, {NUM_CHANNELS}): {channel}")
    hand = "right_ee" if channel < 5 else "left_ee"
    finger = FINGER_NAMES[channel % 5]
    return hand, finger


@dataclass
class TactileMessage:
    channel: int
    frame_id: int
    ts: float
    deform: np.ndarray          # (240, 240) uint8 if deform_len == DEFORM_BYTES, else 1-D
    f6: np.ndarray              # (6,) float32
    contact_point: np.ndarray   # (N*3,) float32, possibly empty

    @property
    def hand_finger(self):
        return channel_to_hand_finger(self.channel)


def pack(channel, frame_id, ts, deform, f6, contact_point):
    """Encode one fingertip frame into a ZMQ message body."""
    deform_bytes = np.ascontiguousarray(deform, dtype=np.uint8).tobytes()
    f6_bytes = np.ascontiguousarray(f6, dtype=np.float32).tobytes()
    cp_bytes = np.ascontiguousarray(contact_point, dtype=np.float32).tobytes()
    header = struct.pack(
        HEADER_FORMAT,
        int(channel),
        int(frame_id),
        float(ts),
        len(deform_bytes),
        len(f6_bytes),
        len(cp_bytes),
    )
    return header + deform_bytes + f6_bytes + cp_bytes


def unpack(payload):
    """Decode a ZMQ message body into a TactileMessage."""
    if len(payload) < HEADER_SIZE:
        raise ValueError(f"payload shorter than header: {len(payload)} < {HEADER_SIZE}")
    channel, frame_id, ts, dlen, flen, clen = struct.unpack_from(HEADER_FORMAT, payload, 0)
    expected = HEADER_SIZE + dlen + flen + clen
    if len(payload) != expected:
        raise ValueError(f"payload size mismatch: got {len(payload)}, expected {expected}")
    o = HEADER_SIZE
    deform = np.frombuffer(payload, dtype=np.uint8, count=dlen, offset=o)
    o += dlen
    f6 = np.frombuffer(payload, dtype=np.float32, count=flen // 4, offset=o)
    o += flen
    cp = np.frombuffer(payload, dtype=np.float32, count=clen // 4, offset=o)
    if dlen == DEFORM_BYTES:
        deform = deform.reshape(DEFORM_H, DEFORM_W)
    return TactileMessage(channel, frame_id, ts, deform, f6, cp)
