"""
Nó Avaliador ("O Juiz Imparcial") — Registro de Séries Temporais e Métricas.

ÚNICO NÓ DO PROJETO AUTORIZADO A ASSINAR /ground_truth/odom (Decisão D3)!
Compara /odom_est contra /ground_truth/odom, calcula e_p(t), e_theta(t), RMSE_p e
RMSE_theta, e salva tudo em CSV na pasta resultados/ para geração de gráficos.
"""

import csv
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from projeto6_navegacao.kinematic_model import quaternion_to_yaw, wrap_to_pi


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger_node")

        default_csv = str(
            Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
            / "resultados"
            / "etapa1"
            / "etapa1_log.csv"
        )
        self.declare_parameter("output_csv", default_csv)

        self.output_csv = Path(str(self.get_parameter("output_csv").value))
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)

        self.ultimo_gt: tuple[float, float, float] | None = None
        self.ultimo_est: tuple[float, float, float] | None = None
        self.t0: float | None = None

        self.linhas: list[list[float]] = []
        self.encerrado = False

        # Assina Ground Truth (exclusivo do avaliador!) e a nossa Odometria estimada
        self.sub_gt = self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, 20)
        self.sub_est = self.create_subscription(Odometry, "/odom_est", self._on_est, 20)
        self.sub_done = self.create_subscription(Bool, "/etapa1_done", self._on_done, 10)

        # Amostragem sincronizada a 20 Hz no relógio de simulação
        self.timer = self.create_timer(0.05, self._on_sample)

        self.get_logger().info(f"MetricsLogger ativo. Salvando em: {self.output_csv}")

    def _on_gt(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.ultimo_gt = (float(p.x), float(p.y), float(yaw))

    def _on_est(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.ultimo_est = (float(p.x), float(p.y), float(yaw))

    def _on_sample(self):
        if self.encerrado or self.ultimo_gt is None or self.ultimo_est is None:
            return

        t_sim = self.get_clock().now().nanoseconds * 1e-9
        if t_sim <= 0.0:
            return
        if self.t0 is None:
            self.t0 = t_sim

        t_rel = t_sim - self.t0
        x_gt, y_gt, th_gt = self.ultimo_gt
        x_est, y_est, th_est = self.ultimo_est

        e_p = math.hypot(x_gt - x_est, y_gt - y_est)
        e_th = abs(wrap_to_pi(th_gt - th_est))

        self.linhas.append([t_rel, x_gt, y_gt, th_gt, x_est, y_est, th_est, e_p, e_th])

    def _on_done(self, msg: Bool):
        if msg.data and not self.encerrado:
            self.encerrado = True
            self.salvar_e_resumir()

    def salvar_e_resumir(self):
        if not self.linhas:
            self.get_logger().warning("Nenhuma amostra coletada para salvar.")
            return

        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["t", "x_gt", "y_gt", "theta_gt", "x_est", "y_est", "theta_est", "e_p", "e_theta"]
            )
            writer.writerows(self.linhas)

        n = len(self.linhas)
        rmse_p = math.sqrt(sum(row[7] ** 2 for row in self.linhas) / n)
        rmse_th = math.sqrt(sum(row[8] ** 2 for row in self.linhas) / n)
        ep_final = self.linhas[-1][7]
        eth_final = self.linhas[-1][8]

        # Comprimento real percorrido segundo o Ground Truth
        comp_gt = 0.0
        for i in range(1, n):
            comp_gt += math.hypot(
                self.linhas[i][1] - self.linhas[i - 1][1],
                self.linhas[i][2] - self.linhas[i - 1][2],
            )

        self.get_logger().info(
            f"\n=== RESUMO ETAPA I ({self.output_csv.name}) ===\n"
            f"  Amostras: {n} | Comprimento real: {comp_gt:.2f} m\n"
            f"  RMSE Posicao    (RMSE_p)     : {rmse_p:.4f} m | Erro final e_p(tf) : {ep_final:.4f} m\n"
            f"  RMSE Orientacao (RMSE_theta) : {math.degrees(rmse_th):.2f} deg | Erro final e_th(tf): {math.degrees(eth_final):.2f} deg\n"
            f"  CSV salvo com sucesso em: {self.output_csv}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.encerrado:
            node.salvar_e_resumir()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
