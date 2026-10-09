import json, os, sys, hashlib
from concurrent.futures import ThreadPoolExecutor
root = sys.argv[1]
jobs = []
for ep in sorted(e for e in os.listdir(root) if e.startswith("episode_")):
    mp = os.path.join(root, ep, "sharpa_native_capture", "manifest.json")
    if not os.path.isfile(mp): print(f"{ep}: no manifest (skipped)"); continue
    m = json.load(open(mp))
    for ch in m["chunks"]:
        jobs.append((ep, os.path.join(root, ep, "sharpa_native_capture", ch["path"]), ch["sha256"], ch.get("stored_bytes")))
def work(j):
    ep, p, want, size = j
    if not os.path.isfile(p): return (ep, p, "MISSING")
    if size is not None and os.path.getsize(p) != size: return (ep, p, f"SIZE {os.path.getsize(p)} != {size}")
    h = hashlib.sha256()
    with open(p, "rb", buffering=1 << 20) as f:
        for b in iter(lambda: f.read(1 << 24), b""): h.update(b)
    return (ep, p, "ok" if h.hexdigest() == want else "SHA MISMATCH")
bad = 0; per_ep = {}
with ThreadPoolExecutor(8) as ex:
    for ep, p, res in ex.map(work, jobs):
        per_ep.setdefault(ep, [0, 0]); per_ep[ep][0] += 1
        if res != "ok": bad += 1; per_ep[ep][1] += 1; print(f"BAD {ep} {os.path.basename(p)}: {res}")
print(f"\nverified {len(jobs)} chunks in {len(per_ep)} episodes, {bad} bad")
for ep, (n, b) in per_ep.items(): print(f"  {ep}: {n} chunks, {b} bad")
