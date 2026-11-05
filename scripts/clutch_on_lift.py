"""Part D: the teleoperation clutch on the lift, on the real mock stack.

A scripted operator hand drives the right arm through a clutch: engaged, it moves
+50 mm forward; released, the lift runs its full stroke at its speed limit;
re-gripped, the hand moves 100 mm down at 50 mm/s, then holds.

Anchoring rules compared (what the target is while released, and what a re-grip
continues from):
  carriage      target held in lift_carriage_link (rides with the lift); the arm is
                not commanded while released. Re-grip continues from that target.
  world_target  target held fixed in base_footprint while released (IK streamed,
                arm moves unattended). Re-grip continues from the HELD target,
                as a target-based clutch does (e.g. dora-openarm-webxr PR #62).
  world_state   as world_target while released, but re-grip continues from the
                MEASURED tip pose (TF), not the held target.

Usage: clutch_on_lift.py OUT.json [rule ...]
"""

import os
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lift_hold import ARM, FRAME, RATE_HZ, Run, dur  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

CARRIAGE = "lift_carriage_link"
REGRIP_VZ = -0.05  # m/s during the 2 s re-grip move; set from argv
START_Q = [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]  # forearm level, tip ~0.78 m at lift 0


def rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def to_pose(p, q):
    m = Pose()
    m.position.x, m.position.y, m.position.z = p
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
    return m


class Clutch(Run):
    def tf_pos(self, frame, child):
        t = self.tf.lookup_transform(frame, child, rclpy.time.Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z]), [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    def solve_in(self, frame, p, q):
        from moveit_msgs.msg import MoveItErrorCodes, RobotState
        from moveit_msgs.srv import GetPositionIK
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "right_arm"
        r.ik_link_name = "openarm_right_ee_base_link"
        r.avoid_collisions = True
        rs = RobotState()
        rs.joint_state.name = list(self.js)
        rs.joint_state.position = list(self.js.values())
        r.robot_state = rs
        r.pose_stamped.header.frame_id = frame
        r.pose_stamped.pose = to_pose(p, q)
        r.timeout.nanosec = int(0.02 * 1e9)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        res = fut.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def send(self, q):
        msg = JointTrajectory()
        msg.joint_names = ARM
        msg.points = [JointTrajectoryPoint(positions=q, time_from_start=dur(1.0 / RATE_HZ))]
        self.arm_pub.publish(msg)

    def episode(self, rule):
        self.reset(START_Q)
        frame = CARRIAGE if rule == "carriage" else FRAME
        tip, quat = self.tf_pos(frame, "openarm_right_ee_base_link")
        log, events = [], {}
        target = tip.copy()
        hand = np.zeros(3)
        t0 = time.monotonic()

        def tick(phase, engaged):
            nonlocal target
            q = None
            if engaged or rule != "carriage":
                q = self.solve_in(frame, target, quat)
                if q is not None:
                    self.send(q)
            w, _ = self.tf_pos(FRAME, "openarm_right_ee_base_link")
            if frame == FRAME:
                tw = target
            else:
                c, cq = self.tf_pos(FRAME, CARRIAGE)
                tw = c + rotate(cq, target)
            log.append([time.monotonic() - t0, phase, self.js["lift_joint"], *hand, *tw, *w, q is not None])

        def run_phase(phase, seconds, engaged, hand_vel=np.zeros(3)):
            nonlocal hand, target
            end = time.monotonic() + seconds
            nxt = time.monotonic()
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.003)
                if time.monotonic() < nxt:
                    continue
                nxt += 1.0 / RATE_HZ
                hand = hand + hand_vel / RATE_HZ
                if engaged:
                    target = anchor_target + (hand - anchor_hand)
                tick(phase, engaged)

        # engaged, hand forward 50 mm at 25 mm/s
        anchor_target, anchor_hand = target.copy(), hand.copy()
        run_phase("engaged_forward", 2.0, True, np.array([0.025, 0.0, 0.0]))
        run_phase("engaged_still", 0.5, True)
        # release; lift full stroke
        events["release_target_world"] = log[-1][7:10]
        stroke = (0.35 - self.js["lift_joint"]) / self.lift_speed
        pending = self.goal(self.lift, ["lift_joint"], [0.35], stroke, wait=False)
        run_phase("released_lift_up", stroke + 0.5, False)
        assert pending.done()
        # re-grip
        w_now, q_now = self.tf_pos(frame, "openarm_right_ee_base_link")
        if rule == "world_state":
            quat[:] = q_now
        held = target.copy()
        anchor_target = w_now.copy() if rule == "world_state" else held
        anchor_hand = hand.copy()
        events["regrip_index"] = len(log)
        events["held_vs_measured_at_regrip_mm"] = float(np.linalg.norm(held - w_now) * 1000) if frame == FRAME else 0.0
        run_phase("engaged_down", 2.0, True, np.array([0.0, 0.0, REGRIP_VZ]))
        run_phase("engaged_hold", 1.0, True)
        self.spin_for(0.3)
        return log, events

    def summarize(self, rule, log, events):
        phases = [row[1] for row in log]
        tw = np.array([row[6:9] for row in log], dtype=float)
        w = np.array([row[9:12] for row in log], dtype=float)
        ok = np.array([row[12] for row in log], dtype=bool)
        i_rel = [i for i, p in enumerate(phases) if p == "released_lift_up"]
        i_eng = [i for i, p in enumerate(phases) if p in ("engaged_down", "engaged_hold")]
        rg = events["regrip_index"]
        steps = np.linalg.norm(np.diff(w, axis=0), axis=1)
        moved = [i for i in i_eng if abs(w[i, 2] - w[rg - 1, 2]) > 0.005]
        return {
            "rule": rule,
            "released: tip moved in world (mm)": round(float(np.linalg.norm(w[i_rel[-1]] - w[i_rel[0] - 1]) * 1000), 1),
            "released: IK failures": int((~ok[i_rel]).sum()) if rule != "carriage" else None,
            "regrip: held target vs measured tip (mm)": round(events["held_vs_measured_at_regrip_mm"], 1),
            "regrip: largest tip step in first 0.25 s (mm)": round(float(steps[rg - 1:rg + 5].max() * 1000), 1),
            "engaged: hand moved z (mm)": round(REGRIP_VZ * 2000, 1),
            "engaged: tip moved z (mm)": round(float((w[i_eng[-1], 2] - w[rg - 1, 2]) * 1000), 1),
            "engaged: dead time before tip moves (s)": round(float(log[moved[0]][0] - log[rg][0]), 2) if moved else None,
            "engaged: IK failures": int((~ok[i_eng]).sum()),
            "engaged: final tip vs target (mm)": round(float(np.linalg.norm(w[i_eng[-1]] - tw[i_eng[-1]]) * 1000), 1),
        }


def main():
    global REGRIP_VZ
    out, vz, *rules = sys.argv[1:]
    REGRIP_VZ = float(vz)
    rclpy.init()
    node = Clutch()
    result = {"regrip_vz_m_s": REGRIP_VZ, "lift_speed_m_s": node.lift_speed, "start_q": START_Q, "rate_hz": RATE_HZ, "episodes": []}
    for rule in rules or ["carriage", "world_target", "world_state"]:
        log, events = node.episode(rule)
        s = node.summarize(rule, log, events)
        print(json.dumps(s), flush=True)
        result["episodes"].append({"summary": s, "events": events, "log": log})
    json.dump(result, open(out, "w"))
    node.goal(node.lift, ["lift_joint"], [0.0], 0.35 / node.lift_speed + 1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
