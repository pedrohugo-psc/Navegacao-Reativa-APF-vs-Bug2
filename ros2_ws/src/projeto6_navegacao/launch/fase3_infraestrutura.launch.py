"""
Launch File da Fase 3 — Infraestrutura Gazebo Harmonic + ros_gz_bridge.

Uso com interface grafica 3D (para voce ver o robo e pilotar no teclado):
    ros2 launch projeto6_navegacao fase3_infraestrutura.launch.py

Uso em modo headless (sem janela grafica, para testes rapidos/automatizados):
    ros2 launch projeto6_navegacao fase3_infraestrutura.launch.py headless:=true
"""

import os

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


def _iniciar_gazebo(context, *args, **kwargs):
    pkg_share = get_package_share_directory("projeto6_navegacao")
    world_name = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    world_path = os.path.join(pkg_share, "worlds", world_name)

    # -r inicia a simulacao rodando; -s executa apenas o servidor fisico (headless)
    cmd = ["gz", "sim", "-r"]
    if headless:
        cmd.append("-s")
    cmd.append(world_path)

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    pkg_share = get_package_share_directory("projeto6_navegacao")
    models_dir = os.path.join(pkg_share, "models")
    bridge_config = os.path.join(pkg_share, "config", "bridge_params.yaml")

    # Garante que o Gazebo encontre model://robo_diferencial
    gz_resource_atual = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    novo_resource_path = f"{models_dir}:{gz_resource_atual}" if gz_resource_atual else models_dir

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="mundo_etapa1_vazio.sdf",
                description="Nome do arquivo .sdf dentro da pasta worlds/ do pacote.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Se true, roda o Gazebo sem interface grafica (-s).",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", novo_resource_path),
            OpaqueFunction(function=_iniciar_gazebo),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
        ]
    )
