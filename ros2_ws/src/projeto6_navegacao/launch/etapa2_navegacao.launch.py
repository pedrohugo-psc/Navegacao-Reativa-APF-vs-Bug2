"""
Launch File da Etapa II (Peso 50%) — Navegação Reativa (APF vs. Bug2) no Gazebo Harmonic.

Exemplos de uso (com interface gráfica 3D para assistir ao vivo!):
  1) APF no Cenário A (Esparsos e Corredor):
     ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_a

  2) APF no Cenário B (Armadilha em "U" -> veja o robô travar no mínimo local!):
     ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_b

  3) Bug2 no Cenário B (Armadilha em "U" -> veja o robô contornar o "U" e vencer!):
     ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=bug2 cenario:=cenario_b

  4) APF no Subcenário GNRON (Parede atrás da meta):
     ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_gnron
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


CENARIOS = {
    "cenario_a": ("cenario_a_esparsos.sdf", 8.0, 0.0),
    "cenario_b": ("cenario_b_armadilha_u.sdf", 6.0, 0.0),
    "cenario_gnron": ("cenario_gnron.sdf", 6.0, 0.0),
}


def _criar_acoes(context, *args, **kwargs):
    pkg_share = get_package_share_directory("projeto6_navegacao")
    bridge_config = os.path.join(pkg_share, "config", "bridge_params.yaml")

    metodo = LaunchConfiguration("metodo").perform(context).lower()
    cenario = LaunchConfiguration("cenario").perform(context).lower()
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    k_att = float(LaunchConfiguration("k_att").perform(context))
    k_rep = float(LaunchConfiguration("k_rep").perform(context))
    rho_0 = float(LaunchConfiguration("rho_0").perform(context))
    enc_noise = float(LaunchConfiguration("encoder_noise_std").perform(context))
    lidar_noise = float(LaunchConfiguration("lidar_noise_std").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))

    if cenario not in CENARIOS:
        raise ValueError(f"Cenario '{cenario}' invalido. Use: {list(CENARIOS.keys())}")
    if metodo not in ("apf", "bug2"):
        raise ValueError(f"Metodo '{metodo}' invalido. Use: apf ou bug2")

    world_file, goal_x, goal_y = CENARIOS[cenario]
    world_path = os.path.join(pkg_share, "worlds", world_file)

    csv_padrao = str(
        Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
        / "resultados"
        / "etapa2"
        / f"{cenario}_{metodo}_seed{seed}.csv"
    )
    output_csv = LaunchConfiguration("output_csv").perform(context) or csv_padrao

    cmd_gz = ["gz", "sim", "-r"]
    if headless:
        cmd_gz.append("-s")
    cmd_gz.append(world_path)

    if metodo == "apf":
        nav_node = Node(
            package="projeto6_navegacao",
            executable="apf_navigator_node",
            name="apf_navigator_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                    "k_att": k_att,
                    "k_rep": k_rep,
                    "rho_0": rho_0,
                    "lidar_noise_std": lidar_noise,
                    "seed": seed,
                }
            ],
            output="screen",
        )
    else:
        nav_node = Node(
            package="projeto6_navegacao",
            executable="bug2_navigator_node",
            name="bug2_navigator_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                    "lidar_noise_std": lidar_noise,
                    "seed": seed,
                }
            ],
            output="screen",
        )

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
                    "encoder_noise_std": enc_noise,
                    "seed": seed,
                }
            ],
            output="screen",
        ),
        nav_node,
        Node(
            package="projeto6_navegacao",
            executable="nav_metrics_logger_node",
            name="nav_metrics_logger_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "output_csv": output_csv,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
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
            DeclareLaunchArgument("metodo", default_value="apf", description="apf ou bug2"),
            DeclareLaunchArgument(
                "cenario", default_value="cenario_a", description="cenario_a, cenario_b ou cenario_gnron"
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("k_att", default_value="1.0"),
            DeclareLaunchArgument("k_rep", default_value="0.8"),
            DeclareLaunchArgument("rho_0", default_value="1.10"),
            DeclareLaunchArgument("encoder_noise_std", default_value="0.05"),
            DeclareLaunchArgument("lidar_noise_std", default_value="0.01"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("output_csv", default_value=""),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", novo_resource_path),
            OpaqueFunction(function=_criar_acoes),
        ]
    )
