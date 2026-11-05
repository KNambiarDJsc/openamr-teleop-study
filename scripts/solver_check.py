"""Is the reach boundary the arm's or the solver's? Re-try the sweep's boundary
points with random seeds and a longer timeout, and measure the configured
kinematics.yaml timeout (5 ms) on points known to be reachable."""

import os
import json
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from reach_sweep import ARM, Probe  # noqa: E402

ORI = "elbow_90_tip"


def main():
    d = json.load(open(sys.argv[1]))
    m = d["meta"]
    xs, ys, zs = m["xs"], m["ys"], m["zs"]
    shape = (len(xs), len(ys), len(zs))
    q = m["orientations"][ORI]
    rclpy.init()
    node = Probe()
    base = node.base_state()
    rng = np.random.default_rng(20261106)
    limits = [(-3.0, 3.0)] * 7  # broad; the solver clamps to URDF limits
    out = {}
    for lift in ("0.0", "0.35"):
        R = np.array(d["reach"][ORI][lift]).reshape(shape)
        grown = R.copy()
        for ax in range(3):
            grown |= np.roll(R, 1, ax) | np.roll(R, -1, ax)
        shell = list(zip(*np.nonzero(grown & ~R)))
        recovered = 0
        for idx in shell:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xs[idx[0]], ys[idx[1]], zs[idx[2]]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            for _ in range(6):
                seed = dict(base, lift_joint=float(lift), **{j: float(rng.uniform(lo, hi)) for j, (lo, hi) in zip(ARM, limits)})
                if node.solve(seed, pose, timeout=0.2):
                    recovered += 1
                    break
        # configured timeout (kinematics.yaml: 0.005 s) from the home seed, on points the sweep solved
        home = dict(base, **{j: 0.0 for j in ARM}, lift_joint=float(lift))
        solved = list(zip(*np.nonzero(R)))
        fast_ok = 0
        for idx in solved:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xs[idx[0]], ys[idx[1]], zs[idx[2]]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            fast_ok += bool(node.solve(home, pose, timeout=0.005))
        out[lift] = {"boundary_points_retried": len(shell), "recovered_with_6_random_seeds_0.2s": recovered,
                     "sweep_solved": len(solved), "solved_from_home_seed_at_5ms": fast_ok}
        print(lift, out[lift], flush=True)
    json.dump(out, open(sys.argv[2], "w"))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
