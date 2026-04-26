import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def _package_share_or_help(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Required package '{package_name}' is not installed. "
            "For ROS 2 Jazzy install Gazebo Sim support with: "
            "sudo apt update && sudo apt install ros-jazzy-ros-gz"
        ) from exc


def generate_launch_description():
    description_share = get_package_share_directory("touch_car_description")
    gazebo_share = get_package_share_directory("touch_car_gazebo")
    ros_gz_sim_share = _package_share_or_help("ros_gz_sim")
    _package_share_or_help("ros_gz_bridge")

    xacro_file = os.path.join(description_share, "urdf", "touch_car.urdf.xacro")
    sdf_file = os.path.join(description_share, "models", "touch_car", "model.sdf")
    world_file = os.path.join(gazebo_share, "worlds", "room_1_5m.world")
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            sdf_file,
            "-name",
            "touch_car",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.03",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/touch_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/touch_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/touch_car/bumper_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        output="screen",
    )

    bumper_driver = Node(
        package="touch_car_control",
        executable="bumper_driver",
        name="bumper_driver",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        bumper_driver,
    ])
