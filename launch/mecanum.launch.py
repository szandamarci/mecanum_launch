import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'mecanum_launch'
    bringup_dir = get_package_share_directory('nav2_bringup')
    local_dir = get_package_share_directory(package_name)
    rviz_dir = os.path.join(local_dir, 'rviz')
    xacro_file = os.path.join(local_dir, 'urdf/mecanum.xacro')
    description_raw = xacro.process_file(xacro_file).toxml()
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    
    robot_state_publisher = Node(package= 'robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             }],
                        output='screen')
    
    joint_state_publisher = Node(package= 'joint_state_publisher',
                        executable='joint_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             }],
                        output='screen')
    
    rviz = Node(package= 'rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', [os.path.join(rviz_dir, 'mecanum.rviz')]],
                output='screen')
    
    start_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch/nav2.launch.py')),
        launch_arguments={
            'map' : map_file,
            'params_file' : params_file,
            'slam' : slam 
        }.items())
    
    joy = Node(package ='joy',
               executable='joy_node',
               name='joy_node',
               parameters=[
                   {
                       'device_id': joy_dev,
                       'deadzone': 0.3,
                       'autorepeat_rate': 20.0,
                   }
               ]
    )

    teleop = Node(package= 'teleop_twist_joy',
                namespace='',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
                remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))}
                )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Path to map file'
        )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(
            local_dir, 'params', 'nav2_params_mecanum.yaml'),
        description='Path to nav2 parameters file'
        )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', 
        default_value='True',
        description='Run slam or not'
        )
    
    declare_joy_vel = DeclareLaunchArgument('joy_vel', default_value='cmd_vel')
    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='xbox')
    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='0')
    declare_publish_stamped_twist = DeclareLaunchArgument('publish_stamped_twist', default_value='false')
    declare_config_filepath = DeclareLaunchArgument('config_filepath', default_value=[
        launch.substitutions.TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])
    print(declare_config_filepath)

    return LaunchDescription([
        declare_joy_vel,
        declare_joy_config,
        declare_joy_dev,
        declare_publish_stamped_twist,
        declare_config_filepath,
        joy,
        teleop,

        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        robot_state_publisher,
        joint_state_publisher, 
        rviz,
        start_nav_cmd

    ])


