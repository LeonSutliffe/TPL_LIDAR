from setuptools import find_packages, setup

package_name = "tilt_axis_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Leon Sutliffe",
    maintainer_email="leon.sutliffe@gmail.com",
    description=(
        "PC-side ROS2 node for the LiDAR scanner's tilt axis, talking to the "
        "MKS SERVO42/57D driver over RS485 via a transparent USB<->RS485 bridge."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "tilt_axis_bridge_node = tilt_axis_bridge.node:main",
        ],
    },
)
