"""Part C: what a released (clutched-out) hand does while the lift moves, on the
real mock stack: their ros2_control controllers, their lift_controller at the
lift's own URDF speed, their move_group IK.

Modes
  base   the held target is fixed relative to the arm mount (what OpenArm's own
         teleop stack does: IK targets are in the robot body frame). The arm is
         not commanded; the lift carries the hand.
  world  the held target is fixed in base_footprint (world: the base does not
         move during manipulation). Each tick, IK at the current lift height is
         streamed to right_arm_controller: the arm moves with nobody holding it.

Usage: lift_hold.py OUT.json START_NAME j1,..,j7 [base|world ...]
"""

import json
import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SIDE = "right"
ARM = [f"openarm_{SIDE}_joint{i}" for i in range(1, 8)]
TIP = f"openarm_{SIDE}_ee_base_link"
FRAME = "base_footprint"
RATE_HZ = 20.0


def dur(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


class Run(Node):
    def __init__(self):
        super().__init__("teleop_lift_hold")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.urdf = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data), qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.arm_pub = self.create_publisher(JointTrajectory, f"/{SIDE}_arm_controller/joint_trajectory", 10)
        self.lift = ActionClient(self, FollowJointTrajectory, "/lift_controller/follow_joint_trajectory")
        self.arm_act = ActionClient(self, FollowJointTrajectory, f"/{SIDE}_arm_controller/follow_joint_trajectory")
        assert self.ik.wait_for_service(timeout_sec=60)
        assert self.lift.wait_for_server(timeout_sec=60)
        assert self.arm_act.wait_for_server(timeout_sec=60)
        while self.urdf is None or not self.js:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.vel_limit, self.lift_speed = {}, None
        for j in ET.fromstring(self.urdf).findall("joint"):
            lim = j.find("limit")
            if lim is not None and lim.get("velocity"):
                if j.get("name") in ARM:
                    self.vel_limit[j.get("name")] = float(lim.get("velocity"))
                if j.get("name") == "lift_joint":
                    self.lift_speed = float(lim.get("velocity"))

    def _js(self, m):
        self.js.update(zip(m.name, m.position))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def tip(self):
        t = self.tf.lookup_transform(FRAME, TIP, rclpy.time.Time()).transform
        return [t.translation.x, t.translation.y, t.translation.z], [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    def goal(self, client, names, positions, seconds, wait=True):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = names
        g.trajectory.points = [JointTrajectoryPoint(positions=[float(p) for p in positions], time_from_start=dur(seconds))]
        fut = client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        assert handle.accepted, "goal rejected"
        res = handle.get_result_async()
        if wait:
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.error_code
        return res

    def reset(self, start):
        """Lift to 0 and arm to `start`, then refuse to go on unless the state shows it got there."""
        self.goal(self.lift, ["lift_joint"], [0.0], max(1.0, self.js.get("lift_joint", 0) / self.lift_speed + 1.0))
        self.goal(self.arm_act, ARM, start, 3.0)
        self.spin_for(1.0)
        err = max(abs(self.js[j] - q) for j, q in zip(ARM, start))
        if err > 0.01 or abs(self.js["lift_joint"]) > 0.001:
            raise RuntimeError(f"start pose not reached (arm error {err:.3f} rad, lift {self.js['lift_joint']:.4f} m): "
                               "is a second controller_manager running on this ROS domain?")

    def solve(self, pose):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = f"{SIDE}_arm"
        r.ik_link_name = TIP
        r.avoid_collisions = True
        rs = RobotState()
        rs.joint_state.name = list(self.js)
        rs.joint_state.position = list(self.js.values())
        r.robot_state = rs
        r.pose_stamped.header.frame_id = FRAME
        r.pose_stamped.pose = pose
        r.timeout.nanosec = int(0.02 * 1e9)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        res = fut.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def hold(self, mode, start, lift_to, label):
        # Reset: lift down, arm to the start pose, settle.
        self.reset(start)
        p0, q0 = self.tip()
        held = Pose()
        held.position.x, held.position.y, held.position.z = p0
        held.orientation.x, held.orientation.y, held.orientation.z, held.orientation.w = q0
        stroke_s = abs(lift_to - self.js["lift_joint"]) / self.lift_speed
        pending = self.goal(self.lift, ["lift_joint"], [lift_to], stroke_s, wait=False)
        samples, fails, last_q, t0 = [], 0, [self.js[j] for j in ARM], time.monotonic()
        peak_ratio = 0.0
        next_tick = t0
        while not pending.done() or time.monotonic() - t0 < stroke_s + 0.5:
            rclpy.spin_once(self, timeout_sec=0.005)
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick += 1.0 / RATE_HZ
            if mode == "world":
                q = self.solve(held)
                if q is None:
                    fails += 1
                else:
                    msg = JointTrajectory()
                    msg.joint_names = ARM
                    msg.points = [JointTrajectoryPoint(positions=q, time_from_start=dur(1.0 / RATE_HZ))]
                    self.arm_pub.publish(msg)
            q_now = [self.js[j] for j in ARM]
            dt = 1.0 / RATE_HZ
            for j, a, b in zip(ARM, last_q, q_now):
                peak_ratio = max(peak_ratio, abs(b - a) / dt / self.vel_limit[j])
            last_q = q_now
            p, _ = self.tip()
            samples.append([now - t0, self.js["lift_joint"], *p, *q_now])
            if now - t0 > stroke_s + 8:
                break
        self.spin_for(0.5)
        p1, _ = self.tip()
        drift = [b - a for a, b in zip(p0, p1)]
        joint_travel = sum(abs(s2[5 + k] - s1[5 + k]) for s1, s2 in zip(samples, samples[1:]) for k in range(7))
        return {"label": label, "mode": mode, "start_q": list(start), "lift_from": 0.0, "lift_to": lift_to,
                "stroke_s": stroke_s, "tip_start": p0, "tip_end": p1, "tip_world_displacement_m": math.dist(p0, p1),
                "tip_delta_xyz_m": drift, "ik_failures": fails, "ticks": len(samples),
                "arm_joint_travel_rad": joint_travel, "peak_joint_speed_fraction_of_limit": peak_ratio,
                "samples": samples}


def main():
    out, name, qtxt, *modes = sys.argv[1:]
    start = [float(v) for v in qtxt.split(",")]
    rclpy.init()
    node = Run()
    print(f"lift speed limit {node.lift_speed} m/s; arm velocity limits {node.vel_limit}", flush=True)
    results = {"lift_speed_m_s": node.lift_speed, "arm_velocity_limits": node.vel_limit, "runs": []}
    for mode in modes or ["base", "world"]:
        r = node.hold(mode, start, 0.35, f"{name}-{mode}")
        r_short = {k: v for k, v in r.items() if k != "samples"}
        print(json.dumps(r_short), flush=True)
        results["runs"].append(r)
    with open(out, "w") as f:
        json.dump(results, f)
    node.goal(node.lift, ["lift_joint"], [0.0], 0.35 / node.lift_speed + 1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
