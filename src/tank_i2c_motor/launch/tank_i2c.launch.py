from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            'tank_i2c_motor'
        ),
        'config',
        'tank.yaml'
    )

    return LaunchDescription([

        Node(
            package='tank_i2c_motor',
            executable='tank_i2c_motor_node',
            name='tank_i2c_motor',
            output='screen',
            parameters=[
                config_file
            ]
        )

    ])
