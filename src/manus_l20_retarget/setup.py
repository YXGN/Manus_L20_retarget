from setuptools import find_packages, setup

package_name = "manus_l20_retarget"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/flexion_right_calibration.yaml",
                "config/finger_yaw_right_calibration.yaml",
                "config/thumb_flexion_mapping.yaml",
                "config/thumb_segment_open_vector_right.yaml",
            ],
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
            "mock_manus_publisher = manus_l20_retarget.mock_manus_publisher:main",
            "manus_somehand_retarget_node = manus_l20_retarget.manus_somehand_retarget_node:main",
            "g20_joint_probe = manus_l20_retarget.g20_joint_probe:main",
            "inspect_manus_landmarks = manus_l20_retarget.inspect_manus_landmarks:main",
            "visualize_thumb_ik = manus_l20_retarget.visualize_thumb_ik:main",
        ],
    },
)
