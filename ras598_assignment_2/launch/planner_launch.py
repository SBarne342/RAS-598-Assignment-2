import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory('ras598_assignment_2')

    map_yaml_path = os.path.join(pkg_share, 'map.yaml')
    cave_image_path = os.path.join(pkg_share, 'cave_filled.png')
    grading_scout_path = os.path.join(pkg_share, 'grading_scout.py')
    rviz_config_path = os.path.join(pkg_share, 'planning.rviz')

    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stage_ros2'),
                'launch',
                'stage.launch.py'
            ])
        ),
        launch_arguments={'world': 'cave'}.items(),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    grading_scout_node = Node(
        package='ras598_assignment_2',
        executable='grading_scout',
        name='grading_scout',
        output='screen',
    )

    planner_node = Node(
        package='ras598_assignment_2',
        executable='planning_core',
        name='planner_node',
        output='screen',
        parameters=[{'map_image_path': cave_image_path}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    delayed_planner = TimerAction(
        period=5.0,
        actions=[planner_node]
    )

    return LaunchDescription([
        stage_launch,
        grading_scout_node,
        map_server_node,
        lifecycle_manager_node,
        delayed_planner,
        rviz_node,
    ])