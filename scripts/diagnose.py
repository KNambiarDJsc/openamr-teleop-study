"""Why does IK fail at the measured tip after a broken world-fixed hold?
Reproduce the end state, then ask move_group: is the current state valid
(collisions), and does IK of the current tip pose solve with and without
collision checking."""

import os
import sys

import rclpy
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clutch_on_lift import START_Q, to_pose  # noqa: E402
from lift_hold import ARM, FRAME, Run  # noqa: E402


def main():
    rclpy.init()
    n = Run()
    valid = n.create_client(GetStateValidity, "/check_state_validity")
    assert valid.wait_for_service(timeout_sec=30)
    n.hold("world", START_Q, 0.35, "diag")  # leaves lift at 0.35, arm frozen at the last solution
    n.spin_for(0.5)

    def state():
        rs = RobotState()
        rs.joint_state.name = list(n.js)
        rs.joint_state.position = list(n.js.values())
        return rs

    req = GetStateValidity.Request()
    req.robot_state = state()
    req.group_name = "right_arm"
    fut = valid.call_async(req)
    rclpy.spin_until_future_complete(n, fut)
    res = fut.result()
    print("lift", round(n.js["lift_joint"], 4), "arm", [round(n.js[j], 3) for j in ARM])
    print("state valid:", res.valid)
    for c in res.contacts[:8]:
        print("  contact:", c.contact_body_1, "<->", c.contact_body_2, "depth", round(c.depth, 4))
    p, q = n.tip()
    for avoid in (True, False):
        r = GetPositionIK.Request()
        r.ik_request.group_name = "right_arm"
        r.ik_request.ik_link_name = "openarm_right_ee_base_link"
        r.ik_request.avoid_collisions = avoid
        r.ik_request.robot_state = state()
        r.ik_request.pose_stamped.header.frame_id = FRAME
        r.ik_request.pose_stamped.pose = to_pose(p, q)
        r.ik_request.timeout.nanosec = int(0.05 * 1e9)
        fut = n.ik.call_async(r)
        rclpy.spin_until_future_complete(n, fut)
        print(f"IK of current tip, avoid_collisions={avoid}:", fut.result().error_code.val == MoveItErrorCodes.SUCCESS,
              "code", fut.result().error_code.val)
    n.goal(n.lift, ["lift_joint"], [0.0], 0.35 / n.lift_speed + 1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
