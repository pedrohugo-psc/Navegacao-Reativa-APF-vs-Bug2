"""
Etapa II — Nó Avaliador de Navegação ("O Juiz Imparcial" da Tabela 1 do Edital).

ÚNICO NÓ AUTORIZADO A ASSINAR /ground_truth/odom na Etapa II (Decisão D3)!
Registra a trajetória real (Ground Truth), a estimativa odométrica (/odom_est),
a distância mínima real aos obstáculos (/scan), colisões, comprimento, tempo,
status final (SUCCESS / LOCAL_MINIMUM / TIMEOUT) e pontos H_i / L_i do Bug2.
"""

import csv
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from projeto6_navegacao.kinematic_model import quaternion_to_yaw, wrap_to_pi


class NavMetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("nav_metrics_logger_node")

        default_csv = str(
            Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
            / "resultados"
            / "etapa2"
            / "etapa2_log.csv"
        )
        self.declare_parameter("output_csv", default_csv)
        self.declare_parameter("collision_radius", 0.21)
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 0.0)

        self.output_csv = Path(str(self.get_parameter("output_csv").value))
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.output_json = self.output_csv.with_suffix(".json")
        self.collision_radius = float(self.get_parameter("collision_radius").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self.ultimo_gt: tuple[float, float, float] | None = None
        self.ultimo_est: tuple[float, float, float] | None = None
        self.ultimo_d_min: float = float("inf")
        self.d_min_global: float = float("inf")
        self.colisoes: int = 0
        self.em_colisao: bool = False
        self.status_nav: str = "RUNNING"
        self.eventos: dict = {"hit_points": [], "leave_points": []}

        self.t0: float | None = None
        self.linhas: list[list[float]] = []
        self.salvo: bool = False

        self.sub_gt = self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, 20)
        self.sub_est = self.create_subscription(Odometry, "/odom_est", self._on_est, 20)
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.sub_status = self.create_subscription(String, "/nav_status", self._on_status, 10)
        self.sub_events = self.create_subscription(String, "/nav_events", self._on_events, 10)

        self.timer = self.create_timer(0.05, self._on_sample)
        self.get_logger().info(f"NavMetricsLogger ativo. Saida: {self.output_csv}")

    def _on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ultimo_gt = (float(p.x), float(p.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def _on_est(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ultimo_est = (float(p.x), float(p.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def _on_scan(self, msg: LaserScan):
        validos = [float(r) for r in msg.ranges if math.isfinite(r) and r >= msg.range_min]
        if not validos:
            return
        d_atual = min(validos)
        self.ultimo_d_min = d_atual
        if d_atual < self.d_min_global:
            self.d_min_global = d_atual
        if d_atual < self.collision_radius:
            if not self.em_colisao:
                self.colisoes += 1
                self.em_colisao = True
        else:
            self.em_colisao = False

    def _on_events(self, msg: String):
        try:
            self.eventos = json.loads(msg.data)
        except Exception:
            pass

    def _on_status(self, msg: String):
        self.status_nav = msg.data
        if self.status_nav != "RUNNING" and not self.salvo:
            self.salvo = True
            self.salvar_resultados()

    def _on_sample(self):
        if self.salvo or self.ultimo_gt is None or self.ultimo_est is None:
            return
        t_sim = self.get_clock().now().nanoseconds * 1e-9
        if t_sim <= 0.0:
            return
        if self.t0 is None:
            self.t0 = t_sim

        t_rel = t_sim - self.t0
        xg, yg, thg = self.ultimo_gt
        xe, ye, the = self.ultimo_est
        e_p = math.hypot(xg - xe, yg - ye)
        e_th = abs(wrap_to_pi(thg - the))
        d_obs = self.ultimo_d_min if math.isfinite(self.ultimo_d_min) else 8.0
        self.linhas.append([t_rel, xg, yg, thg, xe, ye, the, e_p, e_th, d_obs])

    def salvar_resultados(self):
        if not self.linhas:
            return

        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["t", "x_gt", "y_gt", "theta_gt", "x_est", "y_est", "theta_est", "e_p", "e_theta", "d_min_obs"]
            )
            writer.writerows(self.linhas)

        n = len(self.linhas)
        comp_gt = sum(
            math.hypot(self.linhas[i][1] - self.linhas[i - 1][1], self.linhas[i][2] - self.linhas[i - 1][2])
            for i in range(1, n)
        )
        tempo_total = self.linhas[-1][0]
        ep_final = self.linhas[-1][7]
        dist_final_meta_gt = math.hypot(self.goal_x - self.linhas[-1][1], self.goal_y - self.linhas[-1][2])

        resumo = {
            "status": self.status_nav,
            "sucesso": self.status_nav == "SUCCESS",
            "bloqueio_minimo_local": self.status_nav in ("LOCAL_MINIMUM", "TIMEOUT"),
            "colisoes": self.colisoes,
            "comprimento_m": round(comp_gt, 4),
            "tempo_s": round(tempo_total, 3),
            "d_min_m": round(self.d_min_global if math.isfinite(self.d_min_global) else 8.0, 4),
            "erro_final_odom_m": round(ep_final, 4),
            "dist_final_meta_gt_m": round(dist_final_meta_gt, 4),
            "hit_points": self.eventos.get("hit_points", []),
            "leave_points": self.eventos.get("leave_points", []),
        }
        with open(self.output_json, "w") as f:
            json.dump(resumo, f, indent=2)

        self.get_logger().info(
            f"\n=== RESUMO ETAPA II ({self.output_csv.name}) ===\n"
            f"  Status: {resumo['status']} | Colisoes: {resumo['colisoes']} | "
            f"L_traj: {resumo['comprimento_m']:.2f} m | Tempo: {resumo['tempo_s']:.2f} s\n"
            f"  d_min: {resumo['d_min_m']:.3f} m | Erro Odom Final: {resumo['erro_final_odom_m']:.4f} m | "
            f"Dist. Real ate Meta: {resumo['dist_final_meta_gt_m']:.3f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavMetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.salvo:
            node.salvar_resultados()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
