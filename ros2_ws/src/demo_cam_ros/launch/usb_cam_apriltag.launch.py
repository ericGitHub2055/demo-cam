from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params = LaunchConfiguration("params")
    apriltag_yaml = LaunchConfiguration("apriltag_yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value="/work/demo-cam/ros2_ws/src/demo_cam_ros/config/usb_cam.yaml",
        ),
        DeclareLaunchArgument(
            "apriltag_yaml",
            default_value="/work/demo-cam/ros2_ws/src/demo_cam_ros/config/apriltag.yaml",
        ),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[params],
        ),

        Node(
            package="demo_cam_ros",
            executable="image_qos_relay",
            namespace="camera",
            name="image_qos_relay",
            output="screen",
            parameters=[{
                "in_topic": "/camera/image_raw",
                "out_topic": "/camera/image_raw_be",
                "depth": 5,
            }],
        ),

        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            namespace="camera",
            name="apriltag",
            output="screen",
            parameters=[apriltag_yaml],
            remappings=[
                ("image_rect", "image_raw_be"),
                ("camera_info", "camera_info"),
            ],
        ),
    ])
