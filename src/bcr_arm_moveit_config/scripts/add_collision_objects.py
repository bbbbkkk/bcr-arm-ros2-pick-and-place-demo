#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# 创建发布者，向 /planning_scene 话题发布 PlanningScene 消息
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

    def publish_scene(self):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        # 这里先使用 base_link 作为参考坐标系
        frame_id = 'base_link'

        table = self.make_box(
            object_id='table',
            frame_id=frame_id,
            size=[1.2, 1.2, 0.05],
            position=[0.4, 0.0, 0.025]
        )
    

        obstacle_box = self.make_box(
            object_id='box_obstacle',
            frame_id=frame_id,
            size=[0.20, 0.20, 0.40],
            position=[0.45, 0.0, 0.25]
        )
        # 将创建好的的桌子和箱子加入场景
        planning_scene.world.collision_objects.append(table)
        planning_scene.world.collision_objects.append(obstacle_box)

        self.get_logger().info('Publishing collision objects to /planning_scene')

        # 多发布几次，避免第一次 subscriber 还没连接上
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