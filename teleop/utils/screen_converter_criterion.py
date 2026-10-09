"""Apply the LeRobot converter's own tactile-hold criterion to episodes without writing a dataset."""
import sys, os, json, traceback
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, "/home/haochen/Projects_Haochen/unitree_lerobot")
from unitree_lerobot.utils import convert_h2_sharpa_180_to_lerobot_v21 as C
from unitree_lerobot.utils.h2_sharpa_180_alignment import build_causal_tactile_window, target_offsets_ns

TOOLS = C.resolve_capture_tools_dir(None)
MAX_HOLD_MS = 50.0

def screen(ep_dir):
    ep_dir = Path(ep_dir); out = {"episode": f"{ep_dir.parent.name}/{ep_dir.name}"}
    try:
        source, rows = C.load_source_episode(ep_dir)
        capture = C.load_native_capture(ep_dir, TOOLS)
        alignment = C.prepare_episode_alignment(capture, MAX_HOLD_MS)
        max_hold_ns = int(MAX_HOLD_MS * 1e6)
        offsets = target_offsets_ns(C.TACTILE_RATE_HZ, C.TACTILE_WINDOW_SIZE)
        worst_hold = 0; worst_row = None; prefill_rows = 0
        # also measure the raw inter-event gaps per channel (what the hold criterion sees)
        gaps = [np.diff(a) / 1e6 for a in alignment.availability_ns_by_channel]
        out["max_native_event_gap_ms"] = round(float(max(g.max() for g in gaps if g.size)), 1)
        out["native_gaps>50ms"] = int(sum((g > 50).sum() for g in gaps))
        for pos in range(1, len(rows)):
            tick = int(rows[pos]["timestamps"]["workstation_monotonic_ns"])
            w = build_causal_tactile_window(tick, alignment.availability_ns_by_channel, alignment.event_index_by_channel,
                                            alignment.force_by_channel, offsets_ns=offsets, max_hold_ns=max_hold_ns)
            hold = np.where(w.prefill_mask.astype(bool), 0, w.grid_hold_age_ns)
            if w.prefill_mask.any(): prefill_rows += 1
            m = int(hold.max())
            if m > worst_hold: worst_hold, worst_row = m, pos
        out.update(result="PASS", frames=len(rows) - 1, max_hold_ms=round(worst_hold / 1e6, 1), worst_row=worst_row,
                   prefill_rows=prefill_rows, clock_fit_anchors=getattr(alignment.clock_fit, "anchor_count", None))
    except Exception as e:
        out.update(result="FAIL", error=f"{type(e).__name__}: {str(e)[:220]}")
    return out

if __name__ == "__main__":
    eps = []
    for root in sys.argv[1:]:
        eps += sorted(str(p) for p in Path(root).glob("episode_*") if (p / "data.json").is_file())
    with ProcessPoolExecutor(8) as ex:
        results = list(ex.map(screen, eps))
    json.dump(results, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "converter_screen.json"), "w"), indent=1)
    for r in results:
        if r["result"] == "PASS":
            print(f"PASS {r['episode']:28s} frames={r['frames']:4d} max_hold={r['max_hold_ms']:6.1f} ms (row {r['worst_row']})  native max gap={r['max_native_event_gap_ms']:6.1f} ms, gaps>50ms={r['native_gaps>50ms']}")
        else:
            print(f"FAIL {r['episode']:28s} {r.get('error')}  | native max gap={r.get('max_native_event_gap_ms')} ms")
    n = len(results); f = sum(r["result"] == "FAIL" for r in results)
    print(f"\n{n} episodes screened with the converter criterion (max hold {MAX_HOLD_MS} ms): {n-f} pass, {f} fail")
