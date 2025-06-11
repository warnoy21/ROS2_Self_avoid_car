from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess, TimerAction
def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_path('car'),
        'urdf',
        'dabakama_ai.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_path('car'),
        'rviz',
        'display.rviz'
    )
    gazebo_bridge_config_path = os.path.join(
        get_package_share_path('car'),
        'config',
        'gazebo_bridge.yaml'

    )


    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
 
    
      # Launch Gazebo with an empty world
    gz_sim = ExecuteProcess(
       cmd=[
        'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
        f'gz_args:={os.path.join(get_package_share_path("car"), "world", "my_world.sdf")} -r'
        ],

        output='screen'
    )

    # Spawn the robot in Gazebo after a short delay
    spawn_robot = TimerAction(
        period=5.0,  # Delay to ensure Gazebo is ready
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description'],
            output='screen'
        )]
    )

       # Add ROS-Gazebo bridge
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': gazebo_bridge_config_path}]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        ros_gz_bridge_node,
        rviz2_node,
     
    ])
