"""
Launch File da Etapa I (Peso 20%) — Modelagem Cinemática e Odometria RK2.

Exemplos de uso (com interface gráfica 3D do Gazebo para assistir ao vivo):
    ros2 launch projeto6_navegacao etapa1_odometria.launch.py trajetoria:=reta
    ros2 launch projeto6_navegacao etapa1_odometria.launch.py trajetoria:=arco
    ros2 launch projeto6_navegacao etapa1_odometria.launch.py trajetoria:=composto

Em modo headless (sem janela gráfica, mais rápido):
    ros2 launch projeto6_navegacao etapa1_odometria.launch.py trajetoria:=composto headless:=true
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _criar_acoes(context, *args, **kwargs):
    pkg_share = get_package_share_directory("projeto6_navegacao")
    world_path = os.path.join(pkg_share, "worlds", "mundo_etapa1_vazio.sdf")
    bridge_config = os.path.join(pkg_share, "config", "bridge_params.yaml")

    trajetoria = LaunchConfiguration("trajetoria").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    noise_std = float(LaunchConfiguration("encoder_noise_std").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))

    csv_padrao = str(
        Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
        / "resultados"
        / "etapa1"
        / f"etapa1_{trajetoria}.csv"
    )
    output_csv = LaunchConfiguration("output_csv").perform(context) or csv_padrao

    cmd_gz = ["gz", "sim", "-r"]
    if headless:
        cmd_gz.append("-s")
    cmd_gz.append(world_path)

    return [
        ExecuteProcess(cmd=cmd_gz, output="screen"),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[{"config_file": bridge_config}],
            output="screen",
        ),
        Node(
            package="projeto6_navegacao",
            executable="odom_estimator_node",
            name="odom_estimator_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "wheel_radius": 0.05,
                    "wheel_separation": 0.35,
                    "encoder_noise_std": noise_std,
                    "seed": seed,
                }
            ],
            output="screen",
        ),
        Node(
            package="projeto6_navegacao",
            executable="etapa1_test_driver",
            name="etapa1_test_driver",
            parameters=[
                {
                    "use_sim_time": True,
                    "trajetoria": trajetoria,
                }
            ],
            output="screen",
        ),
        Node(
            package="projeto6_navegacao",
            executable="metrics_logger_node",
            name="metrics_logger_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "output_csv": output_csv,
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("projeto6_navegacao")
    models_dir = os.path.join(pkg_share, "models")
    gz_resource_atual = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    novo_resource_path = f"{models_dir}:{gz_resource_atual}" if gz_resource_atual else models_dir

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajetoria",
                default_value="reta",
                description="Trajetoria de teste da Etapa I: reta, arco ou composto.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Se true, roda o Gazebo sem interface grafica.",
            ),
            DeclareLaunchArgument(
                "encoder_noise_std",
                default_value="0.05",
                description="Desvio padrao do ruído gaussiano nos encoders [rad/s].",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="42",
                description="Semente aleatoria fixa para reprodutibilidade.",
            ),
            DeclareLaunchArgument(
                "output_csv",
                default_value="",
                description="Caminho opcional do CSV de saida.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", novo_resource_path),
            OpaqueFunction(function=_criar_acoes),
        ]
    )
