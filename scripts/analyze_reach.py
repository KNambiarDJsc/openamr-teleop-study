import json, sys, numpy as np
d = json.load(open(sys.argv[1]))
m = d["meta"]; xs, ys, zs, lifts = m["xs"], m["ys"], m["zs"], m["lifts"]
print("reference:", {k: (np.round(v, 3).tolist() if isinstance(v, list) else round(v, 3)) for k, v in m["reference"].items()})
shape = (len(xs), len(ys), len(zs))
for oname, table in d["reach"].items():
    R = {float(l): np.array(v).reshape(shape) for l, v in table.items()}
    print(f"\n== {oname}")
    for l in lifts:
        zr = [zs[k] for k in range(len(zs)) if R[l][:, :, k].any()]
        print(f"  lift {l:.4f}: {R[l].sum():4d} pts, tip z band {min(zr):.2f}-{max(zr):.2f} m" if zr else f"  lift {l}: none")
    union = np.any([R[l] for l in lifts], axis=0); allL = np.all([R[l] for l in lifts], axis=0)
    zu = [zs[k] for k in range(len(zs)) if union[:, :, k].any()]
    print(f"  union over lift: {union.sum()} pts, z {min(zu):.2f}-{max(zu):.2f}; reachable at EVERY lift height: {allL.sum()}")
    base = R[0.0]
    # world-fixed hold starting at lift 0: largest lift reached while staying reachable at every sampled height up to it
    surv = []
    for idx in zip(*np.nonzero(base)):
        top = 0.0
        for l in lifts[1:]:
            if R[l][idx]: top = l
            else: break
        surv.append(top)
    surv = np.array(surv)
    print(f"  world-fixed hold from lift 0, raising: of {len(surv)} start points, survives to "
          + ", ".join(f">= {l*1000:.0f} mm: {np.mean(surv >= l)*100:.0f}%" for l in lifts[1:]))
    # same, starting mid-stroke and going both ways (lift 0.175)
    mid = R[0.175]
    both = np.mean([R[0.0][i] and R[0.35][i] for i in zip(*np.nonzero(mid))])
    print(f"  starting at mid-stroke, reachable at both stroke ends: {both*100:.0f}% of {mid.sum()}")
