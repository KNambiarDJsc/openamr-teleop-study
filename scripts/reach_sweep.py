"""Part B: how the lift changes what a teleoperated hand can reach, and how much
lift travel a world-fixed target survives. Uses the running move_group's own
/compute_ik and /compute_fk (their kinematics.yaml, KDL, their URDF limits).

Run with sim:=mock moveit.launch.py up. Writes JSON to argv[1].
"""

import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

SIDE = "right"
ARM = [f"openarm_{SIDE}_joint{i}" for i in range(1, 8)]
TIP = f"openarm_{SIDE}_ee_base_link"
FRAME = "base_footprint"
LIFTS = [0.0, 0.0875, 0.175, 0.2625, 0.35]


class Probe(Node):
    def __init__(self):
        super().__init__("teleop_reach_sweep")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        for c in (self.ik, self.fk):
            assert c.wait_for_service(timeout_sec=60), c.srv_name

    def _js(self, msg):
        self.latest = msg

    def call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def base_state(self):
        while self.latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return dict(zip(self.latest.name, self.latest.position))

    def state(self, joints):
        rs = RobotState()
        rs.joint_state.name = list(joints)
        rs.joint_state.position = [float(v) for v in joints.values()]
        return rs

    def fk_pose(self, joints):
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = [TIP]
        req.robot_state = self.state(joints)
        res = self.call(self.fk, req)
        return res.pose_stamped[0].pose

    def solve(self, joints, pose, timeout=0.05, attempts=1):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = f"{SIDE}_arm"
        r.ik_link_name = TIP
        r.avoid_collisions = True
        r.robot_state = self.state(joints)
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose = pose
        r.pose_stamped = ps
        r.timeout.nanosec = int(timeout * 1e9)
        res = self.call(self.ik, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return {j: sol[j] for j in ARM}


def main():
    out_path = sys.argv[1]
    rclpy.init()
    node = Probe()
    base = node.base_state()

    # Two hand orientations taken from the model, not invented: the tip's
    # orientation with the arm at home (all zero), and with the elbow bent 90
    # degrees forward (the sign that keeps it inside the URDF limit).
    seeds = {}
    home = dict(base, **{j: 0.0 for j in ARM}, lift_joint=0.0)
    seeds["home"] = home
    p_home = node.fk_pose(home)
    bent = None
    for sign in (1, -1):
        cand = dict(home, **{f"openarm_{SIDE}_joint4": sign * math.pi / 2})
        p = node.fk_pose(cand)
        if p.position.z > p_home.position.z + 0.05:
            bent = cand
            break
    bent = bent or dict(home, **{f"openarm_{SIDE}_joint4": math.pi / 2})
    orientations = {"home_tip": p_home.orientation, "elbow_90_tip": node.fk_pose(bent).orientation}
    p_bent = node.fk_pose(bent)
    shoulder_ref = {"home_tip_z_lift0": p_home.position.z, "elbow_tip": [p_bent.position.x, p_bent.position.y, p_bent.position.z]}

    # Grid in front of the right shoulder, in base_footprint (= world: the
    # base does not move during manipulation, AD-30).
    cy = p_home.position.y
    xs = np.round(np.arange(0.15, 0.751, 0.05), 3)
    ys = np.round(cy + np.arange(-0.20, 0.201, 0.20), 3)
    zs = np.round(np.arange(0.50, 1.801, 0.05), 3)

    results = {"meta": {"tip": TIP, "frame": FRAME, "lifts": LIFTS, "xs": xs.tolist(), "ys": ys.tolist(),
                        "zs": zs.tolist(), "orientations": {k: [o.x, o.y, o.z, o.w] for k, o in orientations.items()},
                        "reference": shoulder_ref, "ik_timeout_s": 0.05, "seeds": "home and elbow_90, first success"},
               "reach": {}}
    t0 = time.monotonic()
    for oname, ori in orientations.items():
        table = {}
        for lift in LIFTS:
            ok = []
            for x in xs:
                for y in ys:
                    for z in zs:
                        pose = PoseStamped().pose
                        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
                        pose.orientation = ori
                        sol = None
                        for seed in (home, bent):
                            sol = node.solve(dict(seed, lift_joint=lift), pose)
                            if sol:
                                break
                        ok.append(bool(sol))
            table[str(lift)] = ok
            print(f"{oname} lift={lift:.4f}: {sum(ok)}/{len(ok)} reachable ({time.monotonic() - t0:.0f}s)", flush=True)
        results["reach"][oname] = table
    with open(out_path, "w") as f:
        json.dump(results, f)
    print("WROTE", out_path)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
