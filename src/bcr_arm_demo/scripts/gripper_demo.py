#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperDemo(Node):
    def __init__(self):
        super().__init__('gripper_demo')

        self.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint',
        ]

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

    def send_gripper_goal(self, position: float, duration_sec: float = 1.0) -> bool:
        self.get_logger().info(
            f'Waiting for /gripper_controller/follow_joint_trajectory ...'
        )

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Cannot find /gripper_controller/follow_joint_trajectory action server.'
            )
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(position), float(position)]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(
            f'Sending gripper goal: {position:.3f} m, duration: {duration_sec:.2f} s'
        )

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error('Failed to send goal.')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        self.get_logger().info('Gripper goal accepted. Waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info('Gripper motion succeeded.')
            return True

        self.get_logger().error(
            f'Gripper motion failed. error_code={result.error_code}, '
            f'error_string="{result.error_string}"'
        )
        return False


def parse_args():
    parser = argparse.ArgumentParser(
        description='BCR Arm gripper open / close / grasp demo'
    )

    parser.add_argument(
        'command',
        choices=['open', 'close', 'grasp', 'custom'],
        help='open: 0.035, close: 0.0, grasp: 0.015, custom: use --position'
    )

    parser.add_argument(
        '--position',
        type=float,
        default=None,
        help='Custom gripper joint position in meters. Valid range: 0.0 ~ 0.035'
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=1.0,
        help='Motion duration in seconds'
    )

    return parser.parse_args()


def main():
    args = parse_args()

    preset_positions = {
        'close': 0.000,
        'grasp': 0.015,
        'open': 0.035,
    }

    if args.command == 'custom':
        if args.position is None:
            raise ValueError('custom command requires --position.')
        target_position = args.position
    else:
        target_position = preset_positions[args.command]

    if target_position < 0.0 or target_position > 0.035:
        raise ValueError(
            f'Invalid gripper position: {target_position}. '
            f'Expected range: 0.0 ~ 0.035.'
        )

    rclpy.init()

    node = GripperDemo()

    success = node.send_gripper_goal(
        position=target_position,
        duration_sec=args.duration
    )

    node.destroy_node()
    rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()