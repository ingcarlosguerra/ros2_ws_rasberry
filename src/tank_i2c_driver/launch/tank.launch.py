from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'tank_i2c_driver'
    )

    config_file = os.path.join(
        package_dir,
        'config',
        'tank.yaml'
    )

    return LaunchDescription([

        Node(
            package='tank_i2c_driver',
            executable='tank_i2c_driver_node',
            name='tank_i2c_driver',
            output='screen',
            parameters=[
                config_file
            ]
        )

    ])

