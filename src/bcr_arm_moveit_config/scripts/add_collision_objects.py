#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddCollisionObjects(Node):
    def __init__(self):
        super().__init__('add_collision_objects')

        self.publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

    def make_box(self, object_id, frame_id, size, position):
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = frame_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            float(size[0]),
            float(size[1]),
            float(size[2])
        ]

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def make_color(self, object_id, rgba):
        color = ObjectColor()
        color.id = object_id
        color.color.r = float(rgba[0])
        color.color.g = float(rgba[1])
        color.color.b = float(rgba[2])
        color.color.a = float(rgba[3])
        return color

    def publish_scene(self):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        frame_id = 'base_link'

        # 绿色桌面：和 Gazebo world 同步，Y 方向加大，防止放置区超出桌面
        table = self.make_box(
            object_id='table',
            frame_id=frame_id,
            size=[1.50, 2.42, 0.05],
            position=[1.05, 0.0, 0.025]
        )

        # 紫色木块
        target_object = self.make_box(
            object_id='target_object',
            frame_id=frame_id,
            size=[0.08, 0.08, 0.10],
            position=[0.85, 0.70, 0.10]
        )

        # 蓝色障碍物
        left_wall_obstacle = self.make_box(
            object_id='left_wall_obstacle',
            frame_id=frame_id,
            size=[0.12, 0.55, 0.35],
            position=[0.95, 0.00, 0.225]
        )

        # 天蓝色放置区域：面积扩大为原来的 4 倍
        place_area = self.make_box(
            object_id='place_area',
            frame_id=frame_id,
            size=[0.44, 0.44, 0.01],
            position=[1.27, -0.92, 0.055]
        )

        # 如果 table 会影响 MoveIt 规划，可以继续保持注释
        planning_scene.world.collision_objects.append(table)

        # 物理抓取时，如果 target_object 影响 MoveIt 靠近木块，也可以临时注释掉
        planning_scene.world.collision_objects.append(target_object)

        planning_scene.world.collision_objects.append(left_wall_obstacle)
        planning_scene.world.collision_objects.append(place_area)

        # RViz / MoveIt PlanningScene 颜色
        # 木块：紫色
        planning_scene.object_colors.append(
            self.make_color('target_object', [1.0, 0.0, 1.0, 1.0])
        )

        # 放置区域：天蓝色
        planning_scene.object_colors.append(
            self.make_color('place_area', [0.0, 0.75, 1.0, 1.0])
        )

        # 障碍物：蓝色
        planning_scene.object_colors.append(
            self.make_color('left_wall_obstacle', [0.2, 0.2, 1.0, 1.0])
        )

        # 桌面：绿色；只有 append(table) 后这个颜色才会显示
        planning_scene.object_colors.append(
            self.make_color('table', [0.0, 0.8, 0.0, 1.0])
        )

        self.get_logger().info('Publishing collision objects to /planning_scene')

        for _ in range(5):
            self.publisher.publish(planning_scene)
            time.sleep(0.5)

        self.get_logger().info('Collision objects added.')


def main():
    rclpy.init()

    node = AddCollisionObjects()
    time.sleep(1.0)

    node.publish_scene()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()