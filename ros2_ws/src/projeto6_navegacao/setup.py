from glob import glob
import os
from setuptools import find_packages, setup

package_name = "projeto6_navegacao"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (
            os.path.join("share", package_name, "models", "robo_diferencial"),
            glob("models/robo_diferencial/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pedro Hugo",
    maintainer_email="pedro@ufba.br",
    description="Projeto 6 (PPGEE0060 - UFBA): Odometria RK2, APF e Bug2 em ROS 2 + Gazebo Harmonic.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "odom_estimator_node = projeto6_navegacao.odom_estimator_node:main",
            "etapa1_test_driver = projeto6_navegacao.etapa1_test_driver:main",
            "metrics_logger_node = projeto6_navegacao.metrics_logger_node:main",
            "apf_navigator_node = projeto6_navegacao.apf_navigator_node:main",
            "bug2_navigator_node = projeto6_navegacao.bug2_navigator_node:main",
            "nav_metrics_logger_node = projeto6_navegacao.nav_metrics_logger_node:main",
        ],
    },
)
