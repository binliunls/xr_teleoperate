# G1 Motion Mode (`--motion`)

## Overview

The `--motion` flag in `teleop_hand_and_arm.py` switches the arm controller between two
fundamentally different operating modes:

| | Debug mode (default) | Motion mode (`--motion`) |
|---|---|---|
| DDS topic | `rt/lowcmd` | `rt/arm_sdk` |
| Leg control | Released — legs go limp | Built-in controller keeps running |
| Robot state | Must be hung or sitting | Can stand and walk |
| Thumbstick | Not used | Drives base locomotion |
| Arm override | Full raw joint control | SDK yields arm joints only |

---

## Debug Mode (default, no `--motion`)

### What happens at startup

`MotionSwitcher.Enter_Debug_Mode()` is called before the arm controller is created:

```python
motion_switcher = MotionSwitcher()
status, result = motion_switcher.Enter_Debug_Mode()
```

This calls `MotionSwitcherClient.ReleaseMode()` in a loop until the robot reports no
active mode. It clears the built-in locomotion controller (AI mode / walking mode)
entirely. All joints — including legs — become uncontrolled until the SDK takes over.

### DDS topic: `rt/lowcmd`

The arm controller publishes on `rt/lowcmd` (`kTopicLowCommand_Debug`). This is the
low-level direct joint command channel. Every joint in the robot body is locked at its
current position at init time (high kp/kd), and only the arm joints receive moving
targets from the teleoperation loop.

### Physical requirement

Because the legs go limp, the robot **must be physically supported** — hung from a
rig, sitting, or otherwise unable to fall. Debug mode is not safe for a free-standing
robot.

### Cleanup

`MotionSwitcher.Exit_Debug_Mode()` calls `SelectMode('ai')` to hand control back to
the built-in AI locomotion controller. (Currently commented out in the code.)

---

## Motion Mode (`--motion`)

### What happens at startup

`MotionSwitcher` is **not** called. The built-in locomotion controller continues
running normally — the robot stays in its regular standing/walking state.

A `LocoClientWrapper` is initialised instead (only when `--input-mode controller`):

```python
loco_wrapper = LocoClientWrapper()
```

This wraps `G1LocoClient` for base velocity commands from the VR thumbstick.

### DDS topic: `rt/arm_sdk`

The arm controller publishes on `rt/arm_sdk` (`kTopicLowCommand_Motion`). This is a
separate channel that the robot's motion controller monitors. When it sees commands
here, it yields the arm joints to the SDK while continuing to run leg control itself.

### Handover flag: `kNotUsedJoint0`

The arm SDK protocol uses a special "flag" joint to signal the robot's motion
controller:

| Robot | `kNotUsedJoint0` index |
|---|---|
| G1_29 | 29 |
| G1_23 | 29 |
| H1_2  | 27 |

At the start of the publish loop, this joint's `q` is set to `1.0`:

```python
self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
```

This value of `1.0` tells the robot's motion controller: *"the arm SDK is active,
yield arm joints to me."* As long as this flag remains `1.0`, the SDK owns the arms.

### Thumbstick locomotion

During the main teleoperation loop, the left/right thumbstick values are forwarded to
`LocoClientWrapper.Move()`:

```python
loco_wrapper.Move(
    -tele_data.left_ctrl_thumbstickValue[1] * 0.3,   # vx (forward/back)
    -tele_data.left_ctrl_thumbstickValue[0] * 0.3,   # vy (strafe)
    -tele_data.right_ctrl_thumbstickValue[0] * 0.3,  # vyaw (turn)
)
```

Velocity is capped at `0.3 m/s` to keep motion safe during teleop.

Pressing both thumbsticks simultaneously triggers damping mode (soft emergency stop):

```python
if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
    loco_wrapper.Damp()
```

### Returning to home (`ctrl_dual_arm_go_home`)

When `ctrl_dual_arm_go_home()` is called in motion mode, after the arms reach zero,
the handover flag is ramped smoothly from `1.0` back to `0.0` over ~2 seconds:

```python
for weight in np.linspace(1, 0, num=101):
    self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight
    time.sleep(0.02)
```

This gradual ramp prevents a sudden jerk as the motion controller takes arm control
back from the SDK.

---

## Choosing the right mode

**Use debug mode (default)** when:
- The robot is hung on a rig or sitting on a table
- You want to record arm+hand demonstrations without worrying about balance
- You are doing dataset collection in a controlled environment

**Use `--motion`** when:
- The robot must remain standing or walking during operation
- You want to teleoperate arms while the robot moves its base
- Deploying in an environment where the robot needs to navigate

---

## Robot support matrix

| Robot | `--motion` supported | Notes |
|---|---|---|
| G1_29 | Yes | `kNotUsedJoint0` = index 29 |
| G1_23 | Yes | `kNotUsedJoint0` = index 29 |
| H1_2  | Yes | `kNotUsedJoint0` = index 27 |
| H2    | Yes | `kNotUsedJoint0` = index 31 |
| H1    | No  | Always uses `rt/lowcmd` |
