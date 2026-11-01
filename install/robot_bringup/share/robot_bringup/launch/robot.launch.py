from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    # IMU MPU6050
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mpu6050driver'),
                'launch',
                'mpu6050driver_launch.py'
            )
        )
    )

    # LiDAR RPLidar C1
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    # Driver del robot
    tank_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tank_i2c_driver'),
                'launch',
                'tank.launch.py'
            )
        )
    )

    # Robot Localization EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_localization'),
                'params',
                'ekf.yaml'
            )
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )


    return LaunchDescription([
        imu_launch,
        lidar_launch,
        tank_launch,
        ekf_node,
    ])
