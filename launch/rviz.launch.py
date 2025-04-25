import os
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
    
    local_dir = get_package_share_directory(package_name)
    
    rviz_dir = os.path.join(local_dir, 'rviz')
    xacro_file = os.path.join(local_dir, 'urdf/mecanum.xacro')
    description_raw = xacro.process_file(xacro_file).toxml()
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    robot_state_publisher = Node(package= 'robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             'use_sim_time' : use_sim_time}],
                        output='screen')
    
    joint_state_publisher = Node(package= 'joint_state_publisher',
                        executable='joint_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             'use_sim_time' : use_sim_time}],
                        output='screen')
    

    rviz = Node(package= 'rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', [os.path.join(rviz_dir, 'mecanum.rviz')]],
                output='screen')
    
    return LaunchDescription([
        
        
        robot_state_publisher,
        joint_state_publisher, 
        rviz,
])


