from pathlib import Path

from setuptools import find_packages, setup


package_name = "manus_l20_revo_style"
config_files = sorted(str(path) for path in Path("config").glob("*.yaml"))
launch_files = sorted(str(path) for path in Path("launch").glob("*.launch.py"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", config_files),
        (f"share/{package_name}/launch", launch_files),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="huangzizhe",
    maintainer_email="huangzizhe@example.com",
    description="Revo-style MANUS ergonomics retargeting adapted for LinkerHand L20 commands.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "revo_style_l20_node = manus_l20_revo_style.retarget_node:main",
            "revo_style_l20_mujoco_node = manus_l20_revo_style.mujoco_node:main",
            "revo_style_l20_offline = manus_l20_revo_style.offline_demo:main",
        ],
    },
)
