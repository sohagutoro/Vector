from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('tethered_robot_analysis')
    
    # Запуск Gazebo с пустым миром
    gazebo_world = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', gazebo_world],
        output='screen'
    )
    
    # Загрузка модели робота
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mecanum_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Бридж для одометрии
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/mecanum_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )
    
    # Контроллер робота
    controller_node = Node(
        package='tethered_robot_analysis',
        executable='mecanum_controller',
        output='screen'
    )
    
    # Публикатор URDF
    robot_description = os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_process,
        robot_state_publisher,
        spawn_entity,
        odom_bridge,
        controller_node,
    ])