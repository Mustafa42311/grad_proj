import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    ld06_lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ldlidar_stl_ros2"),
        "launch",
        "ld06.launch.py"
    ))
)


    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="warehouse_firmware",
        executable="mpu6050_driver.py"
    )

    safety_stop = Node(
        package="warehouse_utils",
        executable="safety_stop.py",
        output="screen",
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("warehouse_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        ld06_lidar,
        controller,
        joystick,
        imu_driver_node,
        # safety_stop,
        localization,
        slam
    ])
