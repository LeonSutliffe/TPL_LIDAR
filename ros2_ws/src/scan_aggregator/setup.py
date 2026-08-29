from setuptools import find_packages, setup

package_name = "scan_aggregator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/aggregator.launch.py"]),
        (f"share/{package_name}/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Leon Sutliffe",
    maintainer_email="leon.sutliffe@gmail.com",
    description=(
        "Step-and-stare scan controller: drives tilt_axis_bridge through a "
        "tilt sweep, captures the VLP-16 cloud at each settled stop, "
        "transforms into a common frame via tf2, and merges into a PCD file."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_aggregator_node = scan_aggregator.node:main",
        ],
    },
)
