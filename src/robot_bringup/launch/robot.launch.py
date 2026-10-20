from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # IMU MPU6050
    imu_node = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        output='screen'
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

    return LaunchDescription([
        imu_node,
        lidar_launch,
        tank_launch
    ])
