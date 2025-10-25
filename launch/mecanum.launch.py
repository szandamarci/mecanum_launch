import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    package_name = 'mecanum_moveit_launch'
    bringup_dir = get_package_share_directory('nav2_bringup')
    local_dir = get_package_share_directory(package_name)
    config_dir = get_package_share_directory('mecanum_moveit_config')
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
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )
    moveit_config = (
        MoveItConfigsBuilder("mecanum")
        .robot_description(
            file_path="config/mecanum.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/mecanum.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )


    robot_state_publisher = Node(package= 'robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,},
                             #'use_sim_time' : 'False'
                             moveit_config.robot_description],
                        output='screen')
    
    joint_state_publisher = Node(package= 'joint_state_publisher',
                        executable='joint_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             #'use_sim_time' : 'False'
                             }],
                        output='screen')
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("mecanum_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[]
    )


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mecanum_moveit_simulation_github"),
            "config",
            "rrbot_controllers.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )

    rviz = Node(package= 'rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', [os.path.join(rviz_dir, 'mecanum.rviz')]],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                output='screen')
    
    start_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch/nav2.launch.py')),
        launch_arguments={
            'map' : map_file,
            'params_file' : params_file,
            'slam' : slam, 
            'use_sim_time' : 'False'
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
    
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_dir, 'launch', 'demo.launch.py')))

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
        #demo_launch,
        ros2_control_hardware_type,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        robot_controller_spawner,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        robot_state_publisher,
        joint_state_publisher, 
        rviz,
        start_nav_cmd

    ])


