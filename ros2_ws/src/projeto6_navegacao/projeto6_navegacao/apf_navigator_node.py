"""
Etapa II — Método A: Controlador por Campos Potenciais Artificiais (APF — Khatib, 1986).

REGRA DE OURO DO EDITAL (Decisão D3 do memoria.md):
  Este nó assina EXCLUSIVAMENTE /odom_est e /scan (com qos_profile_sensor_data).
  Ele NUNCA assina /ground_truth/odom!

Publica:
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /nav_status (std_msgs/msg/String): RUNNING | SUCCESS | LOCAL_MINIMUM | TIMEOUT
"""

import math
import random
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from projeto6_navegacao.kinematic_model import quaternion_to_yaw, wrap_to_pi


class ApfNavigatorNode(Node):
    def __init__(self):
        super().__init__("apf_navigator_node")

        # Parâmetros do controlador APF e limites cinemáticos/dinâmicos
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 0.8)
        self.declare_parameter("rho_0", 1.10)
        self.declare_parameter("k_v", 0.25)
        self.declare_parameter("k_w", 1.8)
        self.declare_parameter("v_max", 0.40)
        self.declare_parameter("w_max", 1.20)
        self.declare_parameter("a_max", 0.80)
        self.declare_parameter("alpha_max", 2.50)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("lidar_noise_std", 0.01)
        self.declare_parameter("seed", 42)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.k_att = float(self.get_parameter("k_att").value)
        self.k_rep = float(self.get_parameter("k_rep").value)
        self.rho_0 = float(self.get_parameter("rho_0").value)
        self.k_v = float(self.get_parameter("k_v").value)
        self.k_w = float(self.get_parameter("k_w").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.alpha_max = float(self.get_parameter("alpha_max").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.lidar_noise_std = float(self.get_parameter("lidar_noise_std").value)
        self.rng = random.Random(int(self.get_parameter("seed").value) + 1000)

        self.pose_est: tuple[float, float, float] | None = None
        self.ultimo_scan: LaserScan | None = None
        self.v_anterior = 0.0
        self.w_anterior = 0.0

        self.t_inicio: float | None = None
        self.status = "RUNNING"
        self.historico_pos: deque[tuple[float, float, float]] = deque()  # (t, x, y)

        # Assina APENAS /odom_est e /scan (nunca /ground_truth/odom!)
        self.sub_odom = self.create_subscription(Odometry, "/odom_est", self._on_odom, 20)
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/nav_status", 10)

        self.dt_ctrl = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt_ctrl, self._on_control)

        self.get_logger().info(
            f"APF Navigator iniciado | Meta=({self.goal_x:.2f}, {self.goal_y:.2f}) | "
            f"k_att={self.k_att}, k_rep={self.k_rep}, rho_0={self.rho_0}"
        )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose_est = (float(p.x), float(p.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def _on_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def _publicar_status(self, texto: str):
        m = String()
        m.data = texto
        self.pub_status.publish(m)

    def _on_control(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        if t_now <= 0.0 or self.pose_est is None or self.ultimo_scan is None:
            return

        if self.t_inicio is None:
            self.t_inicio = t_now

        if self.status != "RUNNING":
            self.pub_cmd.publish(Twist())
            self._publicar_status(self.status)
            return

        x, y, theta = self.pose_est
        elapsed = t_now - self.t_inicio
        dist_goal = math.hypot(self.goal_x - x, self.goal_y - y)

        # 1) Condição de chegada à meta
        if dist_goal <= self.goal_tol:
            self.status = "SUCCESS"
            self.pub_cmd.publish(Twist())
            self._publicar_status(self.status)
            self.get_logger().info(f"APF: META ALCANCADA em {elapsed:.2f} s! (d={dist_goal:.3f} m)")
            return

        # 2) Condição de timeout
        if elapsed >= self.timeout_sec:
            self.status = "TIMEOUT"
            self.pub_cmd.publish(Twist())
            self._publicar_status(self.status)
            self.get_logger().warning(f"APF: TIMEOUT apos {elapsed:.2f} s (d={dist_goal:.3f} m)")
            return

        # 3) Detecção de Mínimo Local / GNRON (janela deslizante de 6.0 s)
        self.historico_pos.append((t_now, x, y))
        while self.historico_pos and (t_now - self.historico_pos[0][0]) > 6.0:
            self.historico_pos.popleft()
        if elapsed > 6.5 and len(self.historico_pos) > 10:
            _, x_old, y_old = self.historico_pos[0]
            if math.hypot(x - x_old, y - y_old) < 0.08:
                self.status = "LOCAL_MINIMUM"
                self.pub_cmd.publish(Twist())
                self._publicar_status(self.status)
                self.get_logger().warning(
                    f"APF: MINIMO LOCAL / GNRON detectado em ({x:.2f}, {y:.2f}) "
                    f"aos {elapsed:.2f} s (d_meta={dist_goal:.2f} m)"
                )
                return

        # 4) Força Atrativa: F_att = k_att * (q_goal - q)
        fx = self.k_att * (self.goal_x - x)
        fy = self.k_att * (self.goal_y - y)

        # 5) Força Repulsiva a partir dos feixes válidos do LiDAR 2D
        scan = self.ultimo_scan
        n_feixes = len(scan.ranges)
        d_alpha = (2.0 * math.pi) / max(n_feixes, 1)

        for i, r_raw in enumerate(scan.ranges):
            if not math.isfinite(r_raw):
                continue
            rho = float(r_raw)
            if self.lidar_noise_std > 0.0:
                rho += self.rng.gauss(0.0, self.lidar_noise_std)
            if rho < scan.range_min or rho > self.rho_0:
                continue

            rho_eff = max(rho, 0.14)
            alpha_i = scan.angle_min + i * scan.angle_increment
            ang_global = theta + alpha_i
            mag = self.k_rep * ((1.0 / rho_eff) - (1.0 / self.rho_0)) * (1.0 / (rho_eff ** 2)) * d_alpha

            # Empurra na direção oposta ao feixe (-cos, -sin)
            fx += mag * (-math.cos(ang_global))
            fy += mag * (-math.sin(ang_global))

        norma_f = math.hypot(fx, fy)
        if norma_f < 1e-6:
            v_ref, w_ref = 0.0, 0.0
        else:
            theta_d = math.atan2(fy, fx)
            e_theta = wrap_to_pi(theta_d - theta)
            w_ref = float(np.clip(self.k_w * e_theta, -self.w_max, self.w_max))
            v_ref = float(np.clip(self.k_v * norma_f * max(0.0, math.cos(e_theta)), 0.0, self.v_max))

        # 6) Limitador de aceleração (Slew-Rate Limiter) para suavidade física
        dv_max = self.a_max * self.dt_ctrl
        dw_max = self.alpha_max * self.dt_ctrl
        v_cmd = self.v_anterior + float(np.clip(v_ref - self.v_anterior, -dv_max, dv_max))
        w_cmd = self.w_anterior + float(np.clip(w_ref - self.w_anterior, -dw_max, dw_max))
        self.v_anterior, self.w_anterior = v_cmd, w_cmd

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd.publish(cmd)
        self._publicar_status("RUNNING")


def main(args=None):
    rclpy.init(args=args)
    node = ApfNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
