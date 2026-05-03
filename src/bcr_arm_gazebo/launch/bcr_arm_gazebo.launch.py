import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bcr_arm_gazebo')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')
    # 执行xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('bcr_arm_description'),
            'urdf',
            'bcr_arm.urdf.xacro'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        ),
        # 告诉所有的节点使用gazebo仿真时钟
        'use_sim_time': True
    }
    # 启动gazebo和机器人模型
    # ros_ign_gazebo这里是连接ros2和gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
        ),
        # -r是立即运行仿真
        launch_arguments={
            'ign_args': f'-r {world_path}'
        }.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
    output='screen'
    )
    # 发布机器人模型
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'publish_robot_description': True}
        ]
    )
# 生成机器人,告诉gazebo,从话题robot_description上订阅信息读取模型
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'bcr_arm',
            '-z', '0.05'
        ],
        output='screen'
    )
    # 状态广播器,可以读取所以的关节位置,并发布到/joint_states 话题
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    # 控制脚本就是这里来执行
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    # 夹爪控制器
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,

        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=7.0, actions=[joint_trajectory_controller_spawner]),
        TimerAction(period=8.0, actions=[gripper_controller_spawner]),
    ])