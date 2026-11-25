"""
Etapa I — Gerador das 3 Trajetórias de Teste Obrigatórias (Seção 2.1 do Edital):
  1) reta:     v = 0.25 m/s, w = 0.00 rad/s por 20.0 s (5.0 m em linha reta)
  2) arco:     v = 0.25 m/s, w = 0.25 rad/s por 25.13 s (círculo completo Rc = 1.0 m)
  3) composto: percurso em "S" (reta + curva esquerda + reta + curva direita + reta)

Publica comandos em /cmd_vel usando o relógio de simulação (/clock) e avisa
/etapa1_done = True quando a trajetória termina.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


TRAJETORIAS = {
    "reta": [
        # (v [m/s], w [rad/s], duracao [s])
        (0.25, 0.00, 20.0),
    ],
    "arco": [
        (0.25, 0.25, 25.13),
    ],
    "composto": [
        (0.25,  0.00, 4.0),  # Reta inicial (1.0 m)
        (0.25, +0.35, 4.5),  # Curva esquerda (~90 graus)
        (0.25,  0.00, 3.0),  # Reta intermediária
        (0.25, -0.35, 4.5),  # Curva direita (~90 graus)
        (0.25,  0.00, 4.0),  # Reta final (1.0 m)
    ],
}


class Etapa1TestDriver(Node):
    def __init__(self):
        super().__init__("etapa1_test_driver")

        self.declare_parameter("trajetoria", "reta")
        self.declare_parameter("atraso_inicial", 1.5)

        self.traj_nome = str(self.get_parameter("trajetoria").value).lower()
        if self.traj_nome not in TRAJETORIAS:
            raise ValueError(
                f"Trajetoria '{self.traj_nome}' invalida. Use: {list(TRAJETORIAS.keys())}"
            )
        self.segmentos = TRAJETORIAS[self.traj_nome]
        self.atraso_inicial = float(self.get_parameter("atraso_inicial").value)

        # Constrói tabela acumulada de tempos de cada segmento
        self.cronograma = []
        t_acum = 0.0
        for v, w, dur in self.segmentos:
            t_acum += dur
            self.cronograma.append((t_acum, v, w))
        self.duracao_total = t_acum

        self.t_inicio: float | None = None
        self.finalizado = False

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_done = self.create_publisher(Bool, "/etapa1_done", 10)
        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

        self.get_logger().info(
            f"Driver Etapa I pronto: trajetoria='{self.traj_nome}' "
            f"(duracao={self.duracao_total:.2f} s + atraso={self.atraso_inicial:.1f} s)"
        )

    def _tempo_sim(self) -> float:
        now = self.get_clock().now().nanoseconds
        return now * 1e-9

    def _on_timer(self):
        t_now = self._tempo_sim()
        if t_now <= 0.0:
            # Aguarda o primeiro tick de /clock chegar do Gazebo
            return

        if self.t_inicio is None:
            self.t_inicio = t_now
            self.get_logger().info(f"Iniciando contagem da trajetoria '{self.traj_nome}' em t={t_now:.2f} s")

        elapsed = t_now - self.t_inicio
        cmd = Twist()

        if elapsed < self.atraso_inicial:
            # Robô parado brevemente para estabilizar todos os nós
            self.pub_cmd.publish(cmd)
            return

        t_traj = elapsed - self.atraso_inicial
        if t_traj <= self.duracao_total:
            for t_fim, v, w in self.cronograma:
                if t_traj <= t_fim:
                    cmd.linear.x = float(v)
                    cmd.angular.z = float(w)
                    break
            self.pub_cmd.publish(cmd)
        else:
            # Para o robô e sinaliza conclusão
            self.pub_cmd.publish(cmd)
            if not self.finalizado:
                self.finalizado = True
                self.get_logger().info(f"Trajetoria '{self.traj_nome}' concluida! Enviando /etapa1_done.")
            done_msg = Bool()
            done_msg.data = True
            self.pub_done.publish(done_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Etapa1TestDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante comando zero ao encerrar
        if rclpy.ok():
            node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
