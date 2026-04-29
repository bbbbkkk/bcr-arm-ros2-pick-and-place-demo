#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class BCRArmCLI(Node):
    def __init__(self):
        super().__init__('bcr_arm_cli')

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7'
        ]

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_joint_goal(self, positions, duration_sec=3.0):
        if len(positions) != len(self.joint_names):
            self.get_logger().error(
                f'需要输入 {len(self.joint_names)} 个关节角，当前输入了 {len(positions)} 个'
            )
            return False

        self.get_logger().info('等待 joint_trajectory_controller action server...')

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '找不到 /joint_trajectory_controller/follow_joint_trajectory'
            )
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'发送目标关节角: {positions}')

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error('发送 goal 失败')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('目标被 controller 拒绝')
            return False

        self.get_logger().info('目标已接受，等待执行结果...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()

        if result_response is None:
            self.get_logger().error('没有收到执行结果')
            return False

        result = result_response.result
        self.get_logger().info(f'执行完成，error_code = {result.error_code}')

        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main():
    rclpy.init()

    node = BCRArmCLI()

    poses = {
        # 基础姿态
        'home':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'ready': [0.0, -0.5, 0.8, 0.0, 0.7, 0.0, 0.0],
        'left':  [0.8, -0.4, 0.7, 0.0, 0.5, 0.0, 0.0],
        'right': [-0.8, -0.4, 0.7, 0.0, 0.5, 0.0, 0.0],

        # 无夹爪 pick-place 运动序列用的姿态
        'pre_pick':  [0.3, -0.6, 0.9, 0.0, 0.6, 0.0, 0.0],
        'pick':      [0.3, -0.8, 1.1, 0.0, 0.5, 0.0, 0.0],
        'lift':      [0.3, -0.5, 0.8, 0.0, 0.7, 0.0, 0.0],
        'pre_place': [-0.5, -0.6, 0.9, 0.0, 0.6, 0.0, 0.0],
        'place':     [-0.5, -0.8, 1.1, 0.0, 0.5, 0.0, 0.0],
    }

    sequences = {
        'demo': [
            'home',
            'ready',
            'left',
            'right',
            'ready',
            'home',
        ],

        # 当前没有夹爪，所以这里只做抓取运动轨迹，不做夹爪开合
        'pick_place_no_gripper': [
            'home',
            'ready',
            'pre_pick',
            'pick',
            'lift',
            'pre_place',
            'place',
            'ready',
            'home',
        ],
    }

    print('')
    print('========== BCR Arm Control CLI ==========')
    print('单个姿态：')
    for name in poses.keys():
        print(f'  - {name}')

    print('')
    print('连续动作：')
    for name in sequences.keys():
        print(f'  - {name}')

    print('')
    print('自定义：')
    print('  - custom')
    print('=========================================')
    print('')

    choice = input('请输入姿态名称或动作序列名称: ').strip()

    if choice in poses:
        node.send_joint_goal(poses[choice], duration_sec=3.0)

    elif choice in sequences:
        sequence = sequences[choice]

        node.get_logger().info(f'开始执行动作序列: {choice}')

        for pose_name in sequence:
            node.get_logger().info(f'执行姿态: {pose_name}')
            success = node.send_joint_goal(poses[pose_name], duration_sec=3.0)

            if not success:
                node.get_logger().error(f'姿态 {pose_name} 执行失败，停止序列')
                break

            time.sleep(0.5)

        node.get_logger().info(f'动作序列 {choice} 执行结束')

    elif choice == 'custom':
        print('请输入 7 个关节角，单位是 rad，用空格分隔')
        print('例如：0 -0.5 0.8 0 0.7 0 0')
        values = input('joint values: ').strip().split()

        if len(values) != 7:
            print('错误：必须输入 7 个数')
        else:
            try:
                positions = [float(v) for v in values]
                node.send_joint_goal(positions, duration_sec=3.0)
            except ValueError:
                print('错误：请输入合法数字，例如：0 -0.5 0.8 0 0.7 0 0')

    else:
        print('未知姿态名称或动作序列名称')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()