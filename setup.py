from setuptools import find_packages, setup
from glob import glob

package_name = "handy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pitosalas",
    maintainer_email="pitosalas@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "net_latency = handy.net_latency:main",
            "params = handy.ros2_params:main",
            "tferror = handy.tf_error_detector:main",
            "node1 = handy.node1:main",
        ],
    },
)
