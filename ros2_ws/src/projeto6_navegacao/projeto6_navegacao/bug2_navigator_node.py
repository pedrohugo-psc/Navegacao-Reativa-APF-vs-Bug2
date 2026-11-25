"""
Etapa II — Método B: Algoritmo Bug2 (Lumelsky & Stepanov, 1987).

REGRA DE OURO DO EDITAL (Decisão D3 do memoria.md):
  Este nó assina EXCLUSIVAMENTE /odom_est e /scan (com qos_profile_sensor_data).
  Ele NUNCA assina /ground_truth/odom!

Publica:
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /nav_status (std_msgs/msg/String): RUNNING | SUCCESS | UNREACHABLE | TIMEOUT
  - /nav_events (std_msgs/msg/String): JSON com lista de pontos H_i e L_i
"""

import json
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from projeto6_navegacao.kinematic_model import quaternion_to_yaw, wrap_to_pi


class Bug2NavigatorNode(Node):
    def __init__(self):
        super().__init__("bug2_navigator_node")

        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("d_hit", 0.52)
        self.declare_parameter("d_wall", 0.45)
        self.declare_parameter("delta_m", 0.10)
        self.declare_parameter("epsilon", 0.15)
        self.declare_parameter("d_debounce", 0.45)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("lidar_noise_std", 0.01)
        self.declare_parameter("seed", 42)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.d_hit = float(self.get_parameter("d_hit").value)
        self.d_wall = float(self.get_parameter("d_wall").value)
        self.delta_m = float(self.get_parameter("delta_m").value)
        self.epsilon = float(self.get_parameter("epsilon").value)
        self.d_debounce = float(self.get_parameter("d_debounce").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.lidar_noise_std = float(self.get_parameter("lidar_noise_std").value)
        self.rng = random.Random(int(self.get_parameter("seed").value) + 2000)

        self.pose_est: tuple[float, float, float] | None = None
        self.ultimo_scan: LaserScan | None = None
        self.inicio: tuple[float, float] | None = None
        self.t_inicio: float | None = None

        self.status = "RUNNING"
        self.fsm_state = "GOAL_SEEKING"
        self.hit_point: tuple[float, float] | None = None
        self.dist_hit_goal = float("inf")
        self.arco_contorno = 0.0
        self.pos_anterior: tuple[float, float] | None = None

        self.hit_points: list[list[float]] = []
        self.leave_points: list[list[float]] = []

        # Assina APENAS /odom_est e /scan (nunca /ground_truth/odom!)
        self.sub_odom = self.create_subscription(Odometry, "/odom_est", self._on_odom, 20)
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/nav_status", 10)
        self.pub_events = self.create_publisher(String, "/nav_events", 10)

        self.timer = self.create_timer(0.05, self._on_control)  # 20 Hz

        self.get_logger().info(
            f"Bug2 Navigator iniciado | Meta=({self.goal_x:.2f}, {self.goal_y:.2f}) | "
            f"d_hit={self.d_hit}, d_wall={self.d_wall}"
        )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose_est = (float(p.x), float(p.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def _on_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def _metricas_linha_m(self, x: float, y: float) -> tuple[float, float]:
        xs, ys = self.inicio
        dx = self.goal_x - xs
        dy = self.goal_y - ys
        comp_sq = dx * dx + dy * dy
        comp = math.sqrt(comp_sq)
        if comp < 1e-9:
            return 0.0, 1.0
        d_m = (dx * (y - ys) - dy * (x - xs)) / comp
        lam = ((x - xs) * dx + (y - ys) * dy) / comp_sq
        return d_m, lam

    def _extrair_feixes(self, scan: LaserScan) -> tuple[np.ndarray, np.ndarray]:
        n = len(scan.ranges)
        angs = np.empty(n, dtype=float)
        dists = np.empty(n, dtype=float)
        for i, r_raw in enumerate(scan.ranges):
            angs[i] = wrap_to_pi(scan.angle_min + i * scan.angle_increment)
            if math.isfinite(r_raw) and r_raw >= scan.range_min:
                val = float(r_raw)
                if self.lidar_noise_std > 0.0:
                    val = max(scan.range_min, val + self.rng.gauss(0.0, self.lidar_noise_std))
                dists[i] = min(val, scan.range_max)
            else:
                dists[i] = float(scan.range_max)
        return angs, dists

    def _setor_min(self, angs: np.ndarray, dists: np.ndarray, a_min: float, a_max: float) -> tuple[float, float]:
        mask = (angs >= a_min) & (angs <= a_max)
        if not np.any(mask):
            return 8.0, 0.0
        ds = dists[mask]
        as_ = angs[mask]
        idx = int(np.argmin(ds))
        return float(ds[idx]), float(as_[idx])

    def _caminho_meta_livre(self, x: float, y: float, theta: float, angs: np.ndarray, dists: np.ndarray) -> bool:
        ang_meta_rel = wrap_to_pi(math.atan2(self.goal_y - y, self.goal_x - x) - theta)
        diffs = np.abs(np.array([wrap_to_pi(a - ang_meta_rel) for a in angs]))
        cone = dists[diffs <= math.radians(18.0)]
        return bool(np.min(cone) > self.d_hit + 0.15) if len(cone) > 0 else True

    def _publicar_estado(self):
        ms = String()
        ms.data = self.status
        self.pub_status.publish(ms)

        me = String()
        me.data = json.dumps({"hit_points": self.hit_points, "leave_points": self.leave_points})
        self.pub_events.publish(me)

    def _on_control(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        if t_now <= 0.0 or self.pose_est is None or self.ultimo_scan is None:
            return

        x, y, theta = self.pose_est
        if self.inicio is None:
            self.inicio = (x, y)
            self.t_inicio = t_now

        if self.status != "RUNNING":
            self.pub_cmd.publish(Twist())
            self._publicar_estado()
            return

        elapsed = t_now - self.t_inicio
        dist_goal = math.hypot(self.goal_x - x, self.goal_y - y)

        if dist_goal <= self.goal_tol:
            self.status = "SUCCESS"
            self.pub_cmd.publish(Twist())
            self._publicar_estado()
            self.get_logger().info(
                f"Bug2: META ALCANCADA em {elapsed:.2f} s! (H={len(self.hit_points)}, L={len(self.leave_points)})"
            )
            return

        if elapsed >= self.timeout_sec:
            self.status = "TIMEOUT"
            self.pub_cmd.publish(Twist())
            self._publicar_estado()
            self.get_logger().warning(f"Bug2: TIMEOUT apos {elapsed:.2f} s (d={dist_goal:.3f} m)")
            return

        if self.pos_anterior is not None and self.fsm_state == "WALL_FOLLOWING":
            self.arco_contorno += math.hypot(x - self.pos_anterior[0], y - self.pos_anterior[1])
        self.pos_anterior = (x, y)

        angs, dists = self._extrair_feixes(self.ultimo_scan)
        rho_front, _ = self._setor_min(angs, dists, math.radians(-26.0), math.radians(26.0))

        v_nom = 0.34
        v_wall = 0.26
        w_max = 1.20
        v_cmd, w_cmd = 0.0, 0.0

        if self.fsm_state == "GOAL_SEEKING":
            if rho_front <= self.d_hit:
                self.hit_point = (x, y)
                self.dist_hit_goal = dist_goal
                self.arco_contorno = 0.0
                self.hit_points.append([round(x, 4), round(y, 4)])
                self.fsm_state = "WALL_FOLLOWING"
                self.get_logger().info(
                    f"Bug2: HIT H_{len(self.hit_points)} em ({x:.2f}, {y:.2f}), d(H,T)={dist_goal:.2f} m"
                )
                v_cmd, w_cmd = 0.0, 0.85
            else:
                d_m, _ = self._metricas_linha_m(x, y)
                ang_meta = math.atan2(self.goal_y - y, self.goal_x - x)
                theta_des = ang_meta - 0.5 * math.atan(d_m)
                e_theta = wrap_to_pi(theta_des - theta)
                w_cmd = float(np.clip(2.0 * e_theta, -w_max, w_max))
                v_cmd = v_nom * max(0.0, math.cos(e_theta))
        else:
            # WALL_FOLLOWING: testa LEAVE_CONDITION (4 condições simultâneas)
            d_m, lam = self._metricas_linha_m(x, y)
            saiu_debounce = self.arco_contorno > self.d_debounce
            na_linha_m = abs(d_m) <= self.delta_m and 0.0 <= lam <= 1.0
            mais_perto = dist_goal < (self.dist_hit_goal - self.epsilon)
            livre_para_meta = self._caminho_meta_livre(x, y, theta, angs, dists)

            if saiu_debounce and na_linha_m and mais_perto and livre_para_meta:
                self.leave_points.append([round(x, 4), round(y, 4)])
                self.fsm_state = "GOAL_SEEKING"
                self.get_logger().info(
                    f"Bug2: LEAVE L_{len(self.leave_points)} em ({x:.2f}, {y:.2f}), d(L,T)={dist_goal:.2f} m"
                )
                v_cmd, w_cmd = 0.0, 0.0
            elif (
                self.hit_point is not None
                and self.arco_contorno > 1.5
                and math.hypot(x - self.hit_point[0], y - self.hit_point[1]) < 0.22
            ):
                # Deu a volta completa no obstáculo sem encontrar saída -> meta inalcançável
                self.status = "UNREACHABLE"
                self.pub_cmd.publish(Twist())
                self._publicar_estado()
                self.get_logger().warning("Bug2: Meta declarada UNREACHABLE apos volta completa.")
                return
            elif rho_front < self.d_hit - 0.05:
                # Quina interna / obstáculo frontal imediato -> gira para esquerda
                v_cmd, w_cmd = 0.04, 0.90
            else:
                # Lei Polar Tangencial no hemisfério direito [-150°, -15°]
                rho_min, alpha_min = self._setor_min(angs, dists, math.radians(-150.0), math.radians(-15.0))
                alpha_tan = wrap_to_pi(alpha_min + math.pi / 2.0)
                w_cmd = float(np.clip(1.8 * alpha_tan + 2.5 * (self.d_wall - rho_min), -w_max, w_max))
                v_cmd = v_wall * max(0.25, math.cos(alpha_tan))

        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.pub_cmd.publish(cmd)
        self._publicar_estado()


def main(args=None):
    rclpy.init(args=args)
    node = Bug2NavigatorNode()
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
