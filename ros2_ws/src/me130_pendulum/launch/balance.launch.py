"""Lab 4 -- balance the pendulum UPRIGHT.
    ros2 launch me130_pendulum balance.launch.py deadband:=0.065

The gains default to ZERO, so this is inert until you set them. That is the
point of the lab. Hold the rod upright, zero the angle, set gains, then arm:
To stop:
    ros2 service call /controller_node/disarm std_srvs/srv/Trigger

This uses imu on pendulum for the angle measurement
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    deadband = LaunchConfiguration("deadband")
    motor_sign = LaunchConfiguration("motor_sign")

    return LaunchDescription([
        DeclareLaunchArgument("deadband", default_value="0.0",  # TODO: <--- Change to your deadband value
                              description="from lab 1; 0.0 means no compensation"),
        DeclareLaunchArgument("motor_sign", default_value="1.0"),  # TODO: <--- Change this if your motor is spinning in the wrong direction

        # Students set these. Zero means the controller does nothing when armed,
        # which is the safe state for a first launch.
        DeclareLaunchArgument("kp", default_value="0.0"), #proportional gain # TODO: <--- Modify in step 4 and 6
        DeclareLaunchArgument("ki", default_value="0.0"), #integral gain # TODO: <--- Modify in step 6
        DeclareLaunchArgument("kd", default_value="0.0"), #derivative gain # TODO: <--- Modify in step 6

        DeclareLaunchArgument("arm_window_deg", default_value="5.0",
                              description="how close to the zero arming is allowed"),
        DeclareLaunchArgument("theta_max_deg", default_value="90.0",
                              description="past this the controller disarms itself"),
        DeclareLaunchArgument("log", default_value="false",
                              description="also record a balance_*.csv"),

        Node(package="me130_pendulum", executable="imu_node", name="imu_node",
             output="screen",
             on_exit=Shutdown()),
        Node(package="me130_pendulum", executable="encoder_node", name="encoder_node",
             output="screen",
             on_exit=Shutdown()),
        Node(package="me130_pendulum", executable="motor_node", name="motor_node",
             output="screen",
             parameters=[{"deadband": deadband, "motor_sign": motor_sign}],
             on_exit=Shutdown()),

        Node(package="me130_pendulum", executable="controller_node",
             name="controller_node", output="screen",
             parameters=[{"kp": LaunchConfiguration("kp"),
                          "ki": LaunchConfiguration("ki"),
                          "kd": LaunchConfiguration("kd"),
                          "arm_window_deg": LaunchConfiguration("arm_window_deg"),
                          "theta_max_deg": LaunchConfiguration("theta_max_deg")}]),

        Node(package="me130_pendulum", executable="logger_node", name="logger_node",
             output="screen",
             parameters=[{"prefix": "balance"}],
             condition=IfCondition(LaunchConfiguration("log"))),
    ])
