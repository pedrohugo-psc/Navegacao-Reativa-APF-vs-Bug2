"""
Etapa I (Peso 20%) — Nó ROS 2 Estimador de Pose por Odometria RK2.

REGRA DE OURO DO EDITAL (Decisão D3 do memoria.md):
  Este nó assina EXCLUSIVAMENTE /joint_states (encoders das rodas).
  Ele NUNCA assina /ground_truth/odom!

Publica a pose estimada (x_est, y_est, theta_est) e velocidades (v, w) em /odom_est.
"""

import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from projeto6_navegacao.kinematic_model import passo_odometria_rk2, yaw_to_quaternion


class OdomEstimatorNode(Node):
    def __init__(self):
        super().__init__("odom_estimator_node")

        # Parâmetros cinemáticos e de ruído (configuráveis via launch ou CLI)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.35)
        self.declare_parameter("left_joint_name", "junta_roda_esquerda")
        self.declare_parameter("right_joint_name", "junta_roda_direita")
        self.declare_parameter("encoder_noise_std", 0.05)  # rad/s (0.0 = encoder ideal)
        self.declare_parameter("seed", 42)

        self.r = float(self.get_parameter("wheel_radius").value)
        self.l = float(self.get_parameter("wheel_separation").value)
        self.left_joint = str(self.get_parameter("left_joint_name").value)
        self.right_joint = str(self.get_parameter("right_joint_name").value)
        self.noise_std = float(self.get_parameter("encoder_noise_std").value)
        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        # Estado estimado inicial na origem (0, 0, 0)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Memória da amostra anterior para calcular dt e dphi
        self.ultimo_tempo: float | None = None
        self.ultimo_phi_e: float | None = None
        self.ultimo_phi_d: float | None = None

        # Assina APENAS os encoders das rodas (/joint_states)
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 20
        )
        # Publica a pose estimada em /odom_est
        self.pub_odom = self.create_publisher(Odometry, "/odom_est", 20)

        self.get_logger().info(
            f"OdomEstimator iniciado (R={self.r:.3f} m, L={self.l:.3f} m, "
            f"sigma_encoder={self.noise_std:.3f} rad/s, seed={seed})"
        )

    def _on_joint_states(self, msg: JointState):
        # Localiza os índices das duas rodas pelo nome (não depende da ordem na lista)
        try:
            idx_e = msg.name.index(self.left_joint)
            idx_d = msg.name.index(self.right_joint)
        except ValueError:
            return

        if len(msg.position) <= max(idx_e, idx_d):
            return

        # Tempo exato da simulação vindo do cabeçalho da mensagem (/clock do Gazebo)
        t_atual = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        phi_e = float(msg.position[idx_e])
        phi_d = float(msg.position[idx_d])

        if self.ultimo_tempo is None:
            self.ultimo_tempo = t_atual
            self.ultimo_phi_e = phi_e
            self.ultimo_phi_d = phi_d
            self._publicar_odom(msg.header.stamp, 0.0, 0.0)
            return

        dt = t_atual - self.ultimo_tempo
        if dt <= 1e-6:
            return
        if dt > 0.5:
            # Proteção caso o simulador seja reiniciado
            self.ultimo_tempo = t_atual
            self.ultimo_phi_e = phi_e
            self.ultimo_phi_d = phi_d
            return

        # Velocidade angular efetiva medida pelo encoder no intervalo dt
        w_e = (phi_e - self.ultimo_phi_e) / dt
        w_d = (phi_d - self.ultimo_phi_d) / dt

        # Injeta ruído gaussiano realista nos encoders (se encoder_noise_std > 0)
        if self.noise_std > 0.0:
            w_e += self.rng.gauss(0.0, self.noise_std)
            w_d += self.rng.gauss(0.0, self.noise_std)

        # Integração numérica RK2 (Ponto Médio) — mesma função da Fase 2!
        self.x, self.y, self.theta, v_est, w_est = passo_odometria_rk2(
            self.x, self.y, self.theta, w_d, w_e, dt, self.r, self.l
        )

        self.ultimo_tempo = t_atual
        self.ultimo_phi_e = phi_e
        self.ultimo_phi_d = phi_d

        self._publicar_odom(msg.header.stamp, v_est, w_est)

    def _publicar_odom(self, stamp, v_est: float, w_est: float):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v_est
        odom.twist.twist.angular.z = w_est

        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
