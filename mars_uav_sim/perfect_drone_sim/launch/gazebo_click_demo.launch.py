import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('perfect_drone_sim')
    pkg_prefix = get_package_prefix('perfect_drone_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    super_config_name = LaunchConfiguration('super_config_name')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = Command([
        'xacro ',
        os.path.join(pkg_share, 'urdf', 'super_gazebo_quadrotor.urdf.xacro'),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'super_quadrotor',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.5',
            '-Y', '1.5741',
        ],
        output='screen',
    )

    cloud_transformer = Node(
        package='perfect_drone_sim',
        executable='world_cloud_transformer_node',
        output='screen',
        parameters=[{
            'target_frame': 'world',
            'input_topic': '/gazebo/lidar/points',
            'output_topic': '/cloud_registered',
            'use_sim_time': use_sim_time,
        }],
    )

    super_planner = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            'config_name': super_config_name,
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'super_light.world'),
        ),
        DeclareLaunchArgument('super_config_name', default_value='click_smooth_ros2.yaml'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz2', 'top_down.rviz'),
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                os.path.join(pkg_prefix, 'lib'),
                ':',
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''),
            ],
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        cloud_transformer,
        super_planner,
        rviz,
    ])
