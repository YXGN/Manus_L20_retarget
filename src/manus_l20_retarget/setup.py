from pathlib import Path

from setuptools import find_packages, setup

package_name = "manus_l20_retarget"
config_files = sorted(str(path) for path in Path("config").glob("*.yaml"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            config_files,
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="huangzizhe",
    maintainer_email="huangzizhe@example.com",
    description="Retarget MANUS glove ergonomics to LinkerHand industrial-20 commands.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "manus_l20_retarget_node = manus_l20_retarget.manus_l20_retarget_node:main",
            "debug_tools = manus_l20_retarget.debug_tools:main",
            "l20_simulation = manus_l20_retarget.l20_simulation:main",
        ],
    },
)
