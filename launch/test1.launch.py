import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare three arguments in test1
    test1_arg1 = DeclareLaunchArgument(
        "test1_arg1",
        default_value="default test1_arg1 set in test1",
        description="Test argument 1",
    )

    argx = DeclareLaunchArgument(
        "argx",
        default_value="default argx set in test1",
        description="Test argument x declared in test1",
    )

    # Include test2.launch.py and pass arguments to it
    package_dir = get_package_share_directory("handy")
    test2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "test2.launch.py")
        ),
        launch_arguments={
            "test1_arg1": LaunchConfiguration("test1_arg1"),
            "argx": LaunchConfiguration("argx"),
        }.items(),
    )

    return LaunchDescription(
        [
            test1_arg1,
            test2_launch,
        ]
    )
