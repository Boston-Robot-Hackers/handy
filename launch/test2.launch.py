from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare test1_arg1 (will receive from test1.launch.py)
    test1_arg1 = DeclareLaunchArgument(
        "test1_arg1",
        default_value="not set",
        description="Test argument 1 received from test1",
    )

    # Declare test2_arg2 (declared in test2.launch.py)
    test2_arg2 = DeclareLaunchArgument(
        "test2_arg2",
        default_value="default test2_arg2 set in test2",
        description="Test argument 2 declared in test2",
    )

    # Declare argx (will receive from test1.launch.py)
    argx = DeclareLaunchArgument(
        "argx",
        default_value="not set",
        description="Test argument x received from test1",
    )

    # Launch node1 with parameters
    node1 = Node(
        package="handy",
        executable="node1",
        name="node1",
        parameters=[
            {
                "test1_arg1": LaunchConfiguration("test1_arg1"),
                "test2_arg2": LaunchConfiguration("test2_arg2"),
                "argx": LaunchConfiguration("argx"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            test1_arg1,
            test2_arg2,
            argx,
            node1,
        ]
    )
