#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()

        arm_controller.joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_flex", "wrist_roll"]
        gripper_controller.joint_names = ["grip_left"]

        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()

        # Robust mapping: prefer explicit joint names in the incoming JointState message.
        # If the message contains only a single slider value, assume it controls the gripper.
        pos_map = {}
        if msg.name and len(msg.name) == len(msg.position):
            for n, p in zip(msg.name, msg.position):
                pos_map[n] = p

        # Build arm goal by reading mapped positions (default to current 0.0 if missing)
        arm_goal.positions = [pos_map.get(n, 0.0) for n in arm_controller.joint_names]

        # Determine gripper position: prefer explicit mapping, otherwise if the incoming
        # message contains a single position value treat that as the gripper slider.
        if 'grip_left' in pos_map:
            gripper_goal.positions = [pos_map['grip_left']]
        elif len(msg.position) == 1:
            gripper_goal.positions = [msg.position[0]]
        elif len(msg.position) >= 6:
            # fallback to the 6th element if the message packs arm+gripper positions
            gripper_goal.positions = [msg.position[5]]
        else:
            gripper_goal.positions = [0.0]

        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)

        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main():
    rclpy.init()
    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()