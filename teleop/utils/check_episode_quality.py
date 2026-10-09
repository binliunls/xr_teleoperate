#!/usr/bin/env python3
"""Data-quality check for an xr_teleoperate H2+Sharpa recording task directory.

Usage: python teleop/utils/check_episode_quality.py <task_dir>   (env: tv)
Checks completeness (frames, images, tactile PNGs, arrays, native 180 Hz capture + chunks),
recording correctness (native manifests, drops, source gaps) and timing/delay (frame period,
camera age + duplicates, tactile staleness/age, lowstate/hand-state age, arm-cmd publish latency,
Thor control RTT). Writes dataset_report.json next to this script; use verify_chunks.py for sha256."""
import json, os, sys, math
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from PIL import Image

ROOT = sys.argv[1]
FPS = 30.0
MS = 1e-6  # ns -> ms

def pct(a, p): return float(np.percentile(a, p)) if len(a) else float("nan")

def check_images(paths, expect_size=None):
    missing, bad, sizes_wrong = 0, 0, 0
    for p in paths:
        if not os.path.isfile(p) or os.path.getsize(p) == 0:
            missing += 1; continue
        try:
            with Image.open(p) as im:
                im.verify()
            if expect_size:
                with Image.open(p) as im:
                    if im.size != expect_size: sizes_wrong += 1
        except Exception:
            bad += 1
    return missing, bad, sizes_wrong

def check_episode(ep_dir):
    ep = os.path.basename(ep_dir); r = {"episode": ep, "issues": []}
    iss = r["issues"].append
    try:
        d = json.load(open(os.path.join(ep_dir, "data.json")))
    except Exception as e:
        iss(f"data.json unreadable: {e}"); return r
    frames = d["data"]; N = len(frames); r["frames"] = N
    if N == 0: iss("0 frames"); return r
    idxs = [f["idx"] for f in frames]
    if idxs != list(range(N)): iss("frame idx not contiguous 0..N-1")

    # ---------------- completeness: files
    color_paths, tact_paths = [], []
    n_color_ref, n_tact_ref = 0, 0
    for f in frames:
        for k, p in (f.get("colors") or {}).items():
            color_paths.append(os.path.join(ep_dir, p)); n_color_ref += 1
        for side, fingers in (f.get("tactiles") or {}).items():
            for fn, t in fingers.items():
                tact_paths.append(os.path.join(ep_dir, t["deform"])); n_tact_ref += 1
    n_color_files = len(os.listdir(os.path.join(ep_dir, "colors")))
    n_tact_files = len(os.listdir(os.path.join(ep_dir, "tactiles")))
    r["colors_ref/files"] = f"{n_color_ref}/{n_color_files}"; r["tactiles_ref/files"] = f"{n_tact_ref}/{n_tact_files}"
    if n_color_ref != 3 * N: iss(f"colors referenced {n_color_ref} != 3*N")
    if n_color_files != n_color_ref: iss(f"colors on disk {n_color_files} != referenced {n_color_ref}")
    if n_tact_ref != 10 * N: iss(f"tactiles referenced {n_tact_ref} != 10*N")
    if n_tact_files != n_tact_ref: iss(f"tactiles on disk {n_tact_files} != referenced {n_tact_ref}")
    m, b, _ = check_images(color_paths)
    if m: iss(f"{m} color images missing/empty")
    if b: iss(f"{b} color images corrupt")
    m, b, sw = check_images(tact_paths, (240, 240))
    if m: iss(f"{m} tactile PNGs missing/empty")
    if b: iss(f"{b} tactile PNGs corrupt")
    if sw: iss(f"{sw} tactile PNGs not 240x240")

    # ---------------- completeness: arrays
    bad_shape = 0; nan = 0
    for f in frames:
        for grp, exp in (("states", {"left_arm": 7, "right_arm": 7, "left_ee": 22, "right_ee": 22, "body": 3}),
                         ("actions", {"left_arm": 7, "right_arm": 7, "left_ee": 22, "right_ee": 22, "body": (0, 3)})):
            for part, n in exp.items():
                q = f[grp][part]["qpos"]
                if len(q) not in (n if isinstance(n, tuple) else (n,)): bad_shape += 1
                elif any(x is None or (isinstance(x, float) and math.isnan(x)) for x in q): nan += 1
            for part in ("left_ee", "right_ee"):
                if len(f["actions"][part].get("desired_qpos", [])) != 22: bad_shape += 1
    if bad_shape: iss(f"{bad_shape} state/action arrays with wrong length")
    if nan: iss(f"{nan} state/action arrays with NaN/None")

    # ---------------- timing: frame period
    t = np.array([f["timestamps"]["workstation_monotonic_ns"] for f in frames], dtype=np.int64)
    dt = np.diff(t) * MS
    r["dur_s"] = round(float((t[-1] - t[0]) / 1e9), 1)
    r["period_ms mean/p99/max"] = f"{dt.mean():.1f}/{pct(dt,99):.1f}/{dt.max():.1f}"
    r["gaps>50ms"] = int((dt > 50).sum()); r["gaps>100ms"] = int((dt > 100).sum())
    if r["gaps>100ms"]: iss(f"{r['gaps>100ms']} frame gaps >100 ms (max {dt.max():.0f} ms)")

    # ---------------- timing: cameras
    cam_age = {}; cam_dup = {}; cam_srcgap = {}
    for cam in frames[0]["timestamps"]["cameras"]:
        ages = np.array([f["timestamps"]["workstation_monotonic_ns"] - f["timestamps"]["cameras"][cam]["workstation_receive_monotonic_ns"] for f in frames]) * MS
        stamps = np.array([f["timestamps"]["cameras"][cam]["ros_header_stamp_ns"] for f in frames], dtype=np.int64)
        cam_age[cam] = ages; cam_dup[cam] = int((np.diff(stamps) == 0).sum())
        cam_srcgap[cam] = np.diff(stamps) * MS
    worst = max(cam_age, key=lambda c: cam_age[c].max())
    all_age = np.concatenate(list(cam_age.values()))
    r["cam_age_ms p50/p99/max"] = f"{pct(all_age,50):.0f}/{pct(all_age,99):.0f}/{all_age.max():.0f}"
    r["cam_dup_frames"] = "/".join(str(cam_dup[c]) for c in cam_dup)
    r["cam_src_gap_max_ms"] = "/".join(f"{cam_srcgap[c].max():.0f}" for c in cam_srcgap)
    stale = int((all_age > 100).sum())
    if stale: iss(f"{stale} camera samples older than 100 ms (worst {worst} {cam_age[worst].max():.0f} ms)")
    dups = sum(cam_dup.values())
    if dups > 0.01 * 3 * N: iss(f"{dups} duplicated camera frames (same ROS stamp) = {100*dups/(3*N):.1f}%")
    for c, g in cam_srcgap.items():
        if (g > 100).sum(): iss(f"{c}: {(g>100).sum()} source-stamp gaps >100 ms (camera stall, max {g.max():.0f} ms)")

    # ---------------- timing: lowstate / hand state
    ls_age = np.array([f["timestamps"]["workstation_monotonic_ns"] - f["timestamps"]["h2_lowstate"]["workstation_receive_monotonic_ns"] for f in frames]) * MS
    ticks = np.array([f["timestamps"]["h2_lowstate"]["unitree_tick"] for f in frames])
    r["lowstate_age_ms p99/max"] = f"{pct(ls_age,99):.0f}/{ls_age.max():.0f}"
    rep = int((np.diff(ticks) == 0).sum())
    if rep: iss(f"{rep} frames with repeated lowstate tick (stale robot state)")
    if ls_age.max() > 100: iss(f"lowstate age max {ls_age.max():.0f} ms")
    hs_age = []
    for side in ("left", "right"):
        a = np.array([f["timestamps"]["workstation_monotonic_ns"] - f["timestamps"]["sharpa_hand_state"][side]["workstation_receive_monotonic_ns"] for f in frames]) * MS
        hs_age.append(a)
        if a.max() > 150: iss(f"{side} hand state age max {a.max():.0f} ms")
    hs_age = np.concatenate(hs_age); r["handstate_age_ms p99/max"] = f"{pct(hs_age,99):.0f}/{hs_age.max():.0f}"

    # ---------------- timing: tactile (30 Hz relay samples embedded in frames)
    t_age = []; stale_t = 0; fid_jumps = []
    prev = {}
    for f in frames:
        fm = f["timestamps"]["workstation_monotonic_ns"]
        for side, fingers in f["tactiles"].items():
            for fn, tt in fingers.items():
                t_age.append((fm - tt["workstation_receive_monotonic_ns"]) * MS)
                key = (side, fn)
                if key in prev:
                    dj = tt["frame_id"] - prev[key]
                    fid_jumps.append(dj)
                    if dj == 0: stale_t += 1
                prev[key] = tt["frame_id"]
    t_age = np.array(t_age); fid_jumps = np.array(fid_jumps)
    r["tactile_age_ms p50/p99/max"] = f"{pct(t_age,50):.0f}/{pct(t_age,99):.0f}/{t_age.max():.0f}"
    r["tactile_stale"] = stale_t
    r["tactile_fid_step p50/max"] = f"{pct(fid_jumps,50):.0f}/{fid_jumps.max():.0f}"
    if stale_t > 0.01 * 10 * N: iss(f"{stale_t} stale tactile samples (frame_id unchanged) = {100*stale_t/(10*N):.1f}%")
    if (t_age > 100).sum(): iss(f"{(t_age>100).sum()} tactile samples older than 100 ms (max {t_age.max():.0f})")
    if (fid_jumps > 30).sum(): iss(f"{(fid_jumps>30).sum()} tactile frame_id jumps >30 (180Hz source gap >166 ms)")

    # ---------------- timing: arm command
    gens = np.array([f["timestamps"]["h2_arm_command"]["target"]["target_generation"] for f in frames])
    lp = [f["timestamps"]["h2_arm_command"]["last_publish"] for f in frames]
    pub_lat = np.array([x["workstation_publish_monotonic_ns"] - x["workstation_target_set_monotonic_ns"] for x in lp]) * MS
    r["armcmd_pub_lat_ms p99/max"] = f"{pct(pub_lat,99):.0f}/{pub_lat.max():.0f}"
    gd = np.diff(gens)
    if (gd <= 0).sum(): iss(f"{(gd<=0).sum()} frames with no new arm target (generation stalled)")

    # ---------------- native 180 Hz capture
    nc_path = os.path.join(ep_dir, "sharpa_native_capture.json")
    if not os.path.isfile(nc_path):
        iss("sharpa_native_capture.json missing"); r["native"] = "MISSING"
    else:
        nc = json.load(open(nc_path)); c = nc.get("counts", {})
        r["native"] = f"{nc.get('status')}/{'valid' if nc.get('valid') else 'INVALID'}"
        if nc.get("status") != "complete" or not nc.get("valid"): iss(f"native capture status={nc.get('status')} valid={nc.get('valid')} degraded={nc.get('degraded_reasons')}")
        if c.get("samples_enqueued") != N or c.get("samples_acknowledged") != N: iss(f"native samples enq/ack {c.get('samples_enqueued')}/{c.get('samples_acknowledged')} != frames {N}")
        if c.get("samples_queue_dropped") or c.get("samples_failed") or c.get("requests_failed"): iss(f"native sample drops/fails: {c}")
        cids = {f["native_capture"]["capture_id"] for f in frames}
        if len(cids) != 1 or cids != {nc.get("capture_id")}: iss("per-frame capture_id inconsistent with sharpa_native_capture.json")
        if any(f["native_capture"]["idx"] != f["idx"] for f in frames): iss("per-frame native_capture idx != frame idx")
        cs = [s for s in nc.get("clock_samples", []) if s.get("network_round_trip_ns") is not None]
        if cs:
            rtt = np.array([s["network_round_trip_ns"] for s in cs]) * MS
            off = np.array([s["server_receive_realtime_ns"] - s["workstation_send_realtime_ns"] for s in cs]) * MS
            r["thor_rtt_ms p50/max"] = f"{pct(rtt,50):.1f}/{rtt.max():.1f}"; r["ws-thor_clock_offset_ms spread"] = f"{off.max()-off.min():.1f}"
            if rtt.max() > 100: iss(f"Thor control RTT max {rtt.max():.0f} ms")
        sr = nc.get("stop_reply", {}); sc = sr.get("counts", {})
        if not sr.get("ok") or sr.get("state") != "finalized" or not sr.get("valid"): iss(f"native STOP reply not ok/finalized/valid: {sr.get('error')} {sr.get('invalid_reasons')}")
        if sc.get("received_total") != sc.get("written"): iss(f"native received {sc.get('received_total')} != written {sc.get('written')}")
        # chunk files vs manifest
        ncd = os.path.join(ep_dir, "sharpa_native_capture"); mp = os.path.join(ncd, "manifest.json")
        if not os.path.isfile(mp): iss("native manifest.json missing")
        else:
            m = json.load(open(mp))
            if str(m.get("valid")) != "True" or m.get("invalid_reasons") not in ([], "[]", None): iss(f"manifest valid={m.get('valid')} reasons={m.get('invalid_reasons')}")
            if m.get("writer_error") not in (None, "None"): iss(f"manifest writer_error={m.get('writer_error')}")
            if m.get("stop_reason") != "control_stop": iss(f"manifest stop_reason={m.get('stop_reason')}")
            chunks = m.get("chunks", []); r["chunks"] = len(chunks)
            missing_c = 0; size_bad = 0; recs = 0
            for ch in chunks:
                p = os.path.join(ncd, ch["path"]); recs += ch.get("records", 0)
                if not os.path.isfile(p): missing_c += 1
                elif "stored_bytes" in ch and os.path.getsize(p) != ch["stored_bytes"]: size_bad += 1
            if missing_c: iss(f"{missing_c} native chunk files missing")
            if size_bad: iss(f"{size_bad} native chunk files with wrong size")
            if sc.get("chunks") not in (None, len(chunks)): iss(f"stop_reply chunks {sc.get('chunks')} != manifest {len(chunks)}")
            if sc.get("written") and recs != sc.get("written"): iss(f"manifest chunk records {recs} != written {sc.get('written')}")
            pd0, pd1 = m.get("publisher_drop_count_start"), m.get("publisher_drop_count_end")
            if pd0 is not None and pd1 is not None and int(pd1) - int(pd0) != 0: iss(f"tactile publisher drops during capture: {int(pd1)-int(pd0)}")
            integ = m.get("integrity") or {}
            if isinstance(integ, str):
                try: integ = json.loads(integ.replace("'", '"'))
                except Exception: integ = {}
            r["src_frame_gaps/nonmono"] = f"{integ.get('tactile_source_frame_gaps')}/{integ.get('tactile_source_frame_nonmonotonic')}"
            rates = m.get("rates_hz") or {}
            if isinstance(rates, dict) and rates:
                tr = [v for k, v in rates.items() if k.startswith("tactile_channel")]
                r["tactile_180_rate min/max"] = f"{min(tr):.1f}/{max(tr):.1f}"
                if min(tr) < 170: iss(f"180 Hz tactile channel rate low: min {min(tr):.1f} Hz")
            dur_m = float(m.get("duration_seconds", 0))
            if abs(dur_m - r["dur_s"]) > 1.0: iss(f"native duration {dur_m:.1f}s vs episode {r['dur_s']}s")
    return r

def main():
    eps = sorted(os.path.join(ROOT, e) for e in os.listdir(ROOT) if e.startswith("episode_"))
    with ProcessPoolExecutor(max_workers=min(16, len(eps))) as ex:
        results = list(ex.map(check_episode, eps))
    json.dump(results, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_report.json"), "w"), indent=1)
    cols = ["episode", "frames", "dur_s", "period_ms mean/p99/max", "gaps>50ms", "cam_age_ms p50/p99/max", "cam_dup_frames",
            "cam_src_gap_max_ms", "lowstate_age_ms p99/max", "handstate_age_ms p99/max", "tactile_age_ms p50/p99/max", "tactile_stale",
            "tactile_fid_step p50/max", "armcmd_pub_lat_ms p99/max", "thor_rtt_ms p50/max", "native", "chunks", "src_frame_gaps/nonmono", "tactile_180_rate min/max"]
    print("\t".join(cols))
    for r in results: print("\t".join(str(r.get(c, "")) for c in cols))
    print("\n=== ISSUES")
    n_bad = 0
    for r in results:
        if r["issues"]:
            n_bad += 1; print(f"{r['episode']}:"); [print(f"   - {i}") for i in r["issues"]]
    print(f"\n{len(results)} episodes, {n_bad} with issues, total frames {sum(r.get('frames',0) for r in results)}")

if __name__ == "__main__": main()
