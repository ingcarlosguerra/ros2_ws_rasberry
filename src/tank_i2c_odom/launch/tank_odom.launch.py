from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'tank_i2c_odom'
    )

    config_file = os.path.join(
        package_dir,
        'config',
        'tank_odom.yaml'
    )

    return LaunchDescription([

        Node(
            package='tank_i2c_odom',
            executable='tank_i2c_odom_node',
            name='tank_i2c_odom',
            output='screen',
            parameters=[
                config_file
            ]
        )

    ])
