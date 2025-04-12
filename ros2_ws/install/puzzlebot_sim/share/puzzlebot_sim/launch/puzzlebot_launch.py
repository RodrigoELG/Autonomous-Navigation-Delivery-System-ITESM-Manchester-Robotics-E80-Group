import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    urdf_file_name = 'puzzlebot.urdf'
    pkg_name = 'puzzlebot_sim'

    urdf = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name
    )
    
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Nodo para publicar el URDF
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Nodo para publicar transformaciones
    tf_publisher_node = Node(
        package=pkg_name,
        executable='frame_publisher',
        name='frame_publisher_node',
        output='screen'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_pub_node,
        tf_publisher_node,
        rviz_node,
        # joint_state_publisher_node
    ])
