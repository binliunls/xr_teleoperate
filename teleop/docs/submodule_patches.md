# Required local modifications to the `teleimager` and `televuer` submodules

Branch `haochens/180hz-tactile-capture` depends on two small changes inside Unitree's
submodules that are **not** in the upstream repositories (we have no push rights there).
A fresh clone therefore gets the stock submodule code and silently loses:

| Submodule | Pinned commit | Patch | What it adds | What breaks without it |
|---|---|---|---|---|
| `teleop/teleimager` (`unitreerobotics/teleimager`) | `2aab15d` | `patches/teleimager_receive_timestamps.patch` (+60/-14, `src/teleimager/image_client.py`) | Per-frame `workstation_receive_monotonic_ns` / `workstation_receive_realtime_ns` on `TeleImage`, stamped when the JPEG arrives from Thor and carried through the JPEG ring buffer, the decode queue and the BGR ring buffer (BGR path reports the decoded frame's stamp). | Camera timestamps in every `data.json` frame (`timestamps.cameras.*.workstation_receive_*`), used by the dataset quality checks and the 180 Hz tactile time alignment. Recording raises on the missing attribute. |
| `teleop/televuer` (`unitreerobotics/televuer`) | `b6ed5db` | `patches/televuer_wrist_offsets.patch` (+25/-2, `src/televuer/tv_wrapper.py`) | `TeleVuerWrapper(wrist_offsets=...)`: per-hand translation (meters, controller targetRay frame) from the controller origin to the operator's wrist pivot, applied to the raw pose before the basis change; orientation untouched; hand-tracking mode ignores it. | `teleop_hand_and_arm.py --wrist-offset` (lever-arm compensation for grips mounted on the Sharpa Avatar gloves). `TeleVuerWrapper` rejects the `wrist_offsets` kwarg. |

## Apply after cloning

```bash
git clone --recurse-submodules -b haochens/180hz-tactile-capture https://github.com/binliunls/xr_teleoperate.git
cd xr_teleoperate
git -C teleop/teleimager apply ../../patches/teleimager_receive_timestamps.patch
git -C teleop/televuer   apply ../../patches/televuer_wrist_offsets.patch
```

Both patches are plain `git diff` output against the pinned commits above, so `git apply`
(or `patch -p1`) works; use `git apply --check` first if the submodule pointer has moved.
The submodules are installed editable (`pip install -e teleop/teleimager teleop/televuer`),
so no reinstall is needed after patching.

Verify:

```bash
python -m pytest tests/test_teleimage_timestamps.py -q          # teleimager timestamps
python -c "from televuer.tv_wrapper import TeleVuerWrapper; import inspect; assert 'wrist_offsets' in inspect.signature(TeleVuerWrapper.__init__).parameters; print('televuer ok')"
```

## Re-doing the change by hand (if the patch no longer applies)

**teleimager, `src/teleimager/image_client.py`**
1. `TeleImage`: add `workstation_receive_monotonic_ns` and `workstation_receive_realtime_ns`
   to `__slots__` and to `__init__` (default `None`).
2. In the ZMQ subscriber thread, right after a JPEG is received, take
   `time.monotonic_ns()` and `time.time_ns()` and write the tuple
   `(jpg_bytes, mono, real)` to the JPEG ring buffer and to the BGR decode queue
   (instead of bare bytes).
3. In the decode thread, unpack the tuple, decode, and write
   `(bgr_array, mono, real)` to the BGR ring buffer.
4. In the read path, unpack whichever buffer is read (handle `None`) and pass the
   stamps into `TeleImage`; for the BGR variant use the BGR record's stamps.

**televuer, `src/televuer/tv_wrapper.py`**
1. `TeleVuerWrapper.__init__`: add `wrist_offsets: dict = None`; store
   `{"left": np.asarray(v, float64).reshape(3), "right": ...}` or `None`.
2. In the controller-mode pose path, immediately **before** the basis change,
   for each valid arm: `T[0:3, 3] += T[0:3, 0:3] @ offset[side]` where `T` is the
   raw targetRay pose matrix. Do nothing in hand-tracking mode.

Keeping the patches in this repo is a stopgap. The clean fix is forking both
submodules, pushing these changes there, and re-pointing `.gitmodules`.
