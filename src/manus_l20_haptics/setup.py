from pathlib import Path

from setuptools import find_packages, setup

package_name = "manus_l20_haptics"
config_files = sorted(str(path) for path in Path("config").glob("*.yaml"))
launch_files = sorted(str(path) for path in Path("launch").glob("*.launch.py"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", config_files),
        (f"share/{package_name}/launch", launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="huangzizhe",
    maintainer_email="huangzizhe@example.com",
    description="Bridge LinkerHand L20 tactile force feedback to MANUS glove vibration motors.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "haptic_feedback_node = manus_l20_haptics.haptic_feedback_node:main",
            "tactile_source_node = manus_l20_haptics.tactile_source_node:main",
        ],
    },
)
