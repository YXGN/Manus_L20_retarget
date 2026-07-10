from setuptools import find_packages, setup

package_name = "linker_hand_ros2_sdk"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        package_name: ["LinkerHand/config/*.yaml"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "python-can"],
    zip_safe=True,
    maintainer="linker-robot",
    maintainer_email="linker-robot@todo.todo",
    description="LinkerHand ROS2 SDK package restored from local build output.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "linker_hand_advanced_g20 = linker_hand_ros2_sdk.linker_hand_advanced_g20:main",
        ],
    },
)
