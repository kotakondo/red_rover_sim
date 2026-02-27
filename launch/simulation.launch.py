import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('red_rover_sim')

    use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROVER_NAME', default_value='RR03'),
        description='Robot namespace for all topics'
    )

    # Process xacro to get robot_description, passing robot_namespace
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'red_rover.urdf.xacro']),
        ' robot_namespace:=', LaunchConfiguration('robot_name'),
    ])

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo (headless — no GUI window)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'false',
        }.items()
    )

    # Static TF for back wheels (continuous joints needed by diff_drive,
    # but static TF avoids sim-time sync issues in RViz)
    back_left_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'p3at_back_left_hub', '--child-frame-id', 'p3at_back_left_wheel'],
        parameters=[{'use_sim_time': True}],
    )
    back_right_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'p3at_back_right_hub', '--child-frame-id', 'p3at_back_right_wheel'],
        parameters=[{'use_sim_time': True}],
    )

    # Spawn robot entity in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'red_rover',
            '-x', '0', '-y', '0', '-z', '0.15'
        ],
        output='screen'
    )

    # RViz (conditional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz,
        world_arg,
        robot_name_arg,
        robot_state_pub,
        back_left_wheel_tf,
        back_right_wheel_tf,
        gazebo,
        spawn,
        rviz,
    ])
