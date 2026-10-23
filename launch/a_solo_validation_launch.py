"""A-only acceptance: no A/B mission, release guard, PWM or fleet bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    names = ("config", "receipt", "source_root", "host_profile")
    declarations = [DeclareLaunchArgument(name) for name in names]
    declarations += [DeclareLaunchArgument("execute", default_value="false")]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in names}
    parameters["execute"] = ParameterValue(LaunchConfiguration("execute"), value_type=bool)
    nodes = [Node(package="payload_catch", executable="a_solo_validation",
        arguments=[role], namespace="/", name="a_solo_"+role,
        parameters=[parameters], output="screen", sigterm_timeout="35", sigkill_timeout="5")
        for role in ("supervisor", "mission")]
    return LaunchDescription(declarations+nodes)
