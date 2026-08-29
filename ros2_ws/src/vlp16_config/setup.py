from setuptools import find_packages, setup

package_name = "vlp16_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Sutliffe",
    maintainer_email="leon.sutliffe@gmail.com",
    description=(
        "VLP-16 hardware configuration over the sensor's own web/CGI API "
        "(RPM, return type, FOV, network), plus velodyne_transform_node's "
        "capture-window parameters."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "vlp16_config_node = vlp16_config.node:main",
        ],
    },
)
