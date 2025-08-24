#!/usr/bin/env python3
import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time as RosTime

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

DELTA_RAD = math.radians(20.0)
STEP_SECONDS = 2.0      # time between points (+20, back, -20, back)
REPEATS = 10
START_DELAY = 1.0       # first point must be > 0

def normalize(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class URSweepNode(Node):
    def __init__(self):
        super().__init__("ur5e_sweep_20deg")
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self._latest_js = None
        self._sent = False
        self._timer = self.create_timer(0.1, self._tick)

    def _js_cb(self, msg: JointState):
        self._latest_js = msg

    def _tick(self):
        if self._sent or self._latest_js is None:
            return

        # Map current joint positions
        name_to_pos = {n: p for n, p in zip(self._latest_js.name, self._latest_js.position)}
        try:
            q0 = [float(name_to_pos[j]) for j in UR5E_JOINTS]
        except KeyError as e:
            self.get_logger().warn(f"Waiting for joint in /joint_states: {e}")
            return

        # Build trajectory: (+20 -> back -> -20 -> back) x REPEATS
        traj = JointTrajectory()
        traj.joint_names = UR5E_JOINTS
        traj.header.stamp = self.get_clock().now().to_msg()  # helpful on some setups

        points: List[JointTrajectoryPoint] = []
        t = START_DELAY  # first point strictly > 0

        def add_point(q, t_from_start):
            pt = JointTrajectoryPoint()
            pt.positions = [normalize(a) for a in q]
            pt.time_from_start.sec = int(t_from_start)
            pt.time_from_start.nanosec = int((t_from_start - int(t_from_start)) * 1e9)
            points.append(pt)

        for _ in range(REPEATS):
            # +20°
            add_point([qi + DELTA_RAD for qi in q0], t);            t += STEP_SECONDS
            # back
            add_point(q0, t);                                       t += STEP_SECONDS
            # -20°
            add_point([qi - DELTA_RAD for qi in q0], t);            t += STEP_SECONDS
            # back
            add_point(q0, t);                                       t += STEP_SECONDS

        traj.points = points

        # Publish a few times to avoid missing the window if the controller just started
        for i in range(3):
            self._traj_pub.publish(traj)
            if i == 0:
                self.get_logger().info(
                    f"Published sweep trajectory: {REPEATS} cycles, total ~{int(t)} s."
                )
            time.sleep(0.2)

        self._sent = True  # send once

def main():
    rclpy.init()
    node = URSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
