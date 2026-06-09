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
    +clen   float32[]  joints (OPTIONAL trailer: NUM_JOINTS float32, DEGREES) --
                       the owning hand's joint angles sampled at publish time, so
                       each fingertip frame ships with the contemporaneous hand
                       pose for time-aligned SaTA FK. Absent on legacy packers
                       (sim stub / pre-joint bridge); detected purely by payload
                       length, so the header is UNCHANGED and old messages still
                       decode (joints=None).

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

# Optional per-message joint trailer. The owning hand's joint angles in DEGREES
# (the bridge reads get_joint_position_degree; the consumer converts to radians
# to match read_hand_state / the FK). Fixed length = the Sharpa hand DOF, which
# matches the C++ bridge's NUM_JOINTS. Presence is inferred by payload length.
JOINTS_LEN = 22
JOINTS_BYTES = JOINTS_LEN * 4  # 88

NUM_CHANNELS = 10
# Within-hand channel order is REVERSED relative to the natural thumb→pinky order:
# verified empirically on 2026-05-25 (recorded a right-hand grip, the activity
# showed up under the pinky slot when expected under thumb). Sharpa SDK doesn't
# document the within-hand channel order in its public headers; this tuple is
# the empirical mapping for the hardware on this robot.
FINGER_NAMES = ("pinky", "ring", "middle", "index", "thumb")


def channel_to_hand_finger(channel):
    """Map SDK channel index (0-9) to ('left_ee'|'right_ee', finger_name).

    Sharpa SDK convention (sample/c++/tactile_sample_common.hpp:133):
    channels 0-4 belong to the right hand, 5-9 to the left hand. Within
    each hand, channel % 5 indexes pinky, ring, middle, index, thumb
    (empirically determined — see FINGER_NAMES comment above).
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
    joints: np.ndarray = None   # (NUM_JOINTS,) float32 DEGREES, or None on legacy packers

    @property
    def hand_finger(self):
        return channel_to_hand_finger(self.channel)


def pack(channel, frame_id, ts, deform, f6, contact_point, joints=None):
    """Encode one fingertip frame into a ZMQ message body.

    ``joints`` (optional): the owning hand's joint angles in DEGREES, length
    JOINTS_LEN. When provided, appended as a fixed-size trailer (the header is
    unchanged) so consumers can pair this fingertip's deform with the
    contemporaneous hand pose. Omit it to emit the legacy (deform-only) frame.
    """
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
    body = header + deform_bytes + f6_bytes + cp_bytes
    if joints is not None:
        joints_arr = np.ascontiguousarray(joints, dtype=np.float32)
        if joints_arr.size != JOINTS_LEN:
            raise ValueError(f"joints must have {JOINTS_LEN} entries, got {joints_arr.size}")
        body += joints_arr.tobytes()
    return body


def unpack(payload):
    """Decode a ZMQ message body into a TactileMessage.

    Accepts both the legacy frame (joints=None) and the joint-augmented frame,
    discriminated purely by total length: the optional trailer is exactly
    JOINTS_BYTES of float32 degrees appended after contact_point.
    """
    if len(payload) < HEADER_SIZE:
        raise ValueError(f"payload shorter than header: {len(payload)} < {HEADER_SIZE}")
    channel, frame_id, ts, dlen, flen, clen = struct.unpack_from(HEADER_FORMAT, payload, 0)
    core = HEADER_SIZE + dlen + flen + clen
    if len(payload) == core:
        has_joints = False
    elif len(payload) == core + JOINTS_BYTES:
        has_joints = True
    else:
        raise ValueError(
            f"payload size mismatch: got {len(payload)}, expected {core} "
            f"(no joints) or {core + JOINTS_BYTES} (with joints)"
        )
    o = HEADER_SIZE
    deform = np.frombuffer(payload, dtype=np.uint8, count=dlen, offset=o)
    o += dlen
    f6 = np.frombuffer(payload, dtype=np.float32, count=flen // 4, offset=o)
    o += flen
    cp = np.frombuffer(payload, dtype=np.float32, count=clen // 4, offset=o)
    o += clen
    joints = None
    if has_joints:
        joints = np.frombuffer(payload, dtype=np.float32, count=JOINTS_LEN, offset=o)
    if dlen == DEFORM_BYTES:
        deform = deform.reshape(DEFORM_H, DEFORM_W)
    return TactileMessage(channel, frame_id, ts, deform, f6, cp, joints)
