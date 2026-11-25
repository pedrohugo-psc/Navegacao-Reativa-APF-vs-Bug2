"""
Fase 2 — Passo 2.3: Laboratório Matemático 2D (Sem ROS e Sem Gazebo)
Algoritmo Bug2 (Lumelsky & Stepanov, 1987): Linha-m, Máquina de Estados e Contorno Tangencial.

Reutiliza cinemática/RK2 de passo1_cinematica_rk2.py e o simulador LiDAR de passo2_apf_2d.py!
Execute com:
    python3 passo3_bug2_2d.py
Para abrir a janela interativa do gráfico (além de salvar o PNG):
    python3 passo3_bug2_2d.py --show
"""

import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from passo1_cinematica_rk2 import (
    cinematica_inversa,
    passo_odometria_rk2,
    wrap_to_pi,
)
from passo2_apf_2d import (
    retangulo_para_segmentos,
    simular_lidar_2d,
)


# ==============================================================================
# 1. GEOMETRIA ANALÍTICA DA LINHA-m (m-line)
# ==============================================================================
def metricas_linha_m(
    x: float, y: float, inicio: tuple[float, float], goal: tuple[float, float]
) -> tuple[float, float]:
    """
    Calcula:
      d_m: distância perpendicular assinada do robô até a linha-m [m]
      lam: projeção escalar normalizada ao longo do segmento S -> T (0 em S, 1 em T)
    """
    xs, ys = inicio
    xg, yg = goal
    dx = xg - xs
    dy = yg - ys
    comp_sq = dx * dx + dy * dy
    comp = math.sqrt(comp_sq)
    if comp < 1e-9:
        return 0.0, 1.0

    d_m = (dx * (y - ys) - dy * (x - xs)) / comp
    lam = ((x - xs) * dx + (y - ys) * dy) / comp_sq
    return d_m, lam


# ==============================================================================
# 2. CONTROLADOR BUG2 (MÁQUINA DE ESTADOS FINITA)
#    (Esta lógica será portada para bug2_navigator_node.py na Fase 5!)
# ==============================================================================
class ControladorBug2:
    """Implementa a FSM do Bug2: GOAL_SEEKING <-> WALL_FOLLOWING."""

    def __init__(
        self,
        inicio: tuple[float, float],
        goal: tuple[float, float],
        d_hit: float = 0.50,
        d_wall: float = 0.45,
        delta_m: float = 0.08,
        epsilon: float = 0.15,
        d_debounce: float = 0.40,
    ):
        self.inicio = inicio
        self.goal = goal
        self.d_hit = d_hit
        self.d_wall = d_wall
        self.delta_m = delta_m
        self.epsilon = epsilon
        self.d_debounce = d_debounce

        self.estado = "GOAL_SEEKING"
        self.hit_point: tuple[float, float] | None = None
        self.dist_hit_goal: float = float("inf")
        self.arco_contorno: float = 0.0
        self.pos_anterior: tuple[float, float] | None = None

        # Histórico dos pontos de impacto (H_i) e saída (L_i) para análise/gráficos
        self.hit_points: list[tuple[float, float]] = []
        self.leave_points: list[tuple[float, float]] = []

    def _distancia_no_setor(
        self, angulos_rel: np.ndarray, distancias: np.ndarray, ang_min: float, ang_max: float
    ) -> tuple[float, float]:
        """Retorna (rho_min, alpha_min) dos feixes dentro do setor [ang_min, ang_max]."""
        mascara = (angulos_rel >= ang_min) & (angulos_rel <= ang_max)
        dists_setor = distancias[mascara]
        angs_setor = angulos_rel[mascara]
        idx = int(np.argmin(dists_setor))
        return float(dists_setor[idx]), float(angs_setor[idx])

    def _caminho_meta_livre(
        self, x: float, y: float, theta: float, angulos_rel: np.ndarray, distancias: np.ndarray
    ) -> bool:
        """Verifica se o cone de +-18 graus apontando para a meta está livre."""
        ang_meta_global = math.atan2(self.goal[1] - y, self.goal[0] - x)
        ang_meta_rel = wrap_to_pi(ang_meta_global - theta)
        diferencas = np.abs(np.array([wrap_to_pi(a - ang_meta_rel) for a in angulos_rel]))
        cone = distancias[diferencas <= math.radians(18.0)]
        return bool(np.min(cone) > self.d_hit + 0.15) if len(cone) > 0 else True

    def passo(
        self, x: float, y: float, theta: float, angulos_rel: np.ndarray, distancias: np.ndarray
    ) -> tuple[float, float]:
        """Executa um ciclo de decisão da FSM do Bug2 e retorna (v, w)."""
        v_nom = 0.32
        v_wall = 0.25
        w_max = 1.20

        # Acumula arco percorrido desde o último ponto de impacto
        if self.pos_anterior is not None and self.estado == "WALL_FOLLOWING":
            self.arco_contorno += math.hypot(x - self.pos_anterior[0], y - self.pos_anterior[1])
        self.pos_anterior = (x, y)

        rho_front, _ = self._distancia_no_setor(
            angulos_rel, distancias, math.radians(-26.0), math.radians(26.0)
        )
        dist_goal = math.hypot(self.goal[0] - x, self.goal[1] - y)

        # ----------------------------------------------------------------------
        # ESTADO 1: GOAL_SEEKING (Seguir a linha-m rumo à meta)
        # ----------------------------------------------------------------------
        if self.estado == "GOAL_SEEKING":
            if rho_front <= self.d_hit:
                # HIT_CONDITION: registra H_i e muda para contorno pela direita
                self.hit_point = (x, y)
                self.dist_hit_goal = dist_goal
                self.arco_contorno = 0.0
                self.hit_points.append(self.hit_point)
                self.estado = "WALL_FOLLOWING"
                return 0.0, 0.8  # Começa a girar para a esquerda (parede fica à direita)

            d_m, _ = metricas_linha_m(x, y, self.inicio, self.goal)
            ang_meta = math.atan2(self.goal[1] - y, self.goal[0] - x)
            # Pequena correção para permanecer sobre a linha-m
            theta_des = ang_meta - 0.5 * math.atan(d_m)
            e_theta = wrap_to_pi(theta_des - theta)

            w_cmd = float(np.clip(2.0 * e_theta, -w_max, w_max))
            v_cmd = v_nom * max(0.0, math.cos(e_theta))
            return v_cmd, w_cmd

        # ----------------------------------------------------------------------
        # ESTADO 2: WALL_FOLLOWING (Contorno mantendo parede à direita)
        # ----------------------------------------------------------------------
        d_m, lam = metricas_linha_m(x, y, self.inicio, self.goal)

        # Testa as 4 condições simultâneas da LEAVE_CONDITION:
        saiu_do_impacto = self.arco_contorno > self.d_debounce
        sobre_linha_m = abs(d_m) <= self.delta_m and 0.0 <= lam <= 1.0
        mais_perto_que_hi = dist_goal < (self.dist_hit_goal - self.epsilon)
        meta_desobstruida = self._caminho_meta_livre(x, y, theta, angulos_rel, distancias)

        if saiu_do_impacto and sobre_linha_m and mais_perto_que_hi and meta_desobstruida:
            self.leave_points.append((x, y))
            self.estado = "GOAL_SEEKING"
            return 0.0, 0.0

        # Quina interna / obstáculo bem à frente: gira para a esquerda no próprio eixo
        if rho_front < self.d_hit - 0.05:
            return 0.04, 0.90

        # Lei Polar Tangencial no hemisfério direito [-150 graus, -15 graus]
        rho_min, alpha_min = self._distancia_no_setor(
            angulos_rel, distancias, math.radians(-150.0), math.radians(-15.0)
        )
        alpha_tan = wrap_to_pi(alpha_min + math.pi / 2.0)

        k_tan = 1.8
        k_dist = 2.5
        w_cmd = float(np.clip(k_tan * alpha_tan + k_dist * (self.d_wall - rho_min), -w_max, w_max))
        v_cmd = v_wall * max(0.25, math.cos(alpha_tan))
        return v_cmd, w_cmd


# ==============================================================================
# 3. SIMULAÇÃO DE UM CENÁRIO COMPLETO COM BUG2
# ==============================================================================
def rodar_cenario_bug2(
    nome: str,
    inicio: tuple[float, float, float],
    goal: tuple[float, float],
    caixas: list[tuple[float, float, float, float]],
    circulos: list[tuple[float, float, float]],
    tempo_max: float = 90.0,
) -> dict:
    dt = 0.05  # 20 Hz
    passos_max = int(tempo_max / dt)
    d_tol = 0.15
    raio_robo = 0.20

    segmentos = []
    for cx, cy, w, h in caixas:
        segmentos.extend(retangulo_para_segmentos(cx, cy, w, h))

    bug2 = ControladorBug2(inicio=(inicio[0], inicio[1]), goal=goal)
    x, y, theta = inicio
    traj = [(x, y, theta)]
    d_min_global = float("inf")
    status = "TIMEOUT"

    for _ in range(passos_max):
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal <= d_tol:
            status = "SUCESSO"
            break

        angulos, dists = simular_lidar_2d(x, y, theta, segmentos, circulos)
        d_min_atual = float(np.min(dists))
        d_min_global = min(d_min_global, d_min_atual)

        if d_min_atual < raio_robo:
            status = "COLISAO"
            break

        v_cmd, w_cmd = bug2.passo(x, y, theta, angulos, dists)
        w_d, w_e = cinematica_inversa(v_cmd, w_cmd)
        x, y, theta = passo_odometria_rk2(x, y, theta, w_d, w_e, dt)
        traj.append((x, y, theta))

    traj_arr = np.array(traj)
    diffs = np.diff(traj_arr[:, :2], axis=0)
    comprimento = float(np.sum(np.linalg.norm(diffs, axis=1))) if len(diffs) > 0 else 0.0
    dist_final_meta = math.hypot(goal[0] - traj_arr[-1, 0], goal[1] - traj_arr[-1, 1])

    print(f"[{nome}]")
    print(
        f"  Status: {status} | Distância final: {dist_final_meta:.3f} m | "
        f"Comprimento: {comprimento:.2f} m | d_min: {d_min_global:.2f} m | "
        f"Hits H_i: {len(bug2.hit_points)} | Leaves L_i: {len(bug2.leave_points)}"
    )

    return {
        "nome": nome,
        "traj": traj_arr,
        "inicio": inicio,
        "goal": goal,
        "caixas": caixas,
        "circulos": circulos,
        "status": status,
        "comprimento": comprimento,
        "hit_points": bug2.hit_points,
        "leave_points": bug2.leave_points,
    }


def desenhar_cenario_bug2(ax, res: dict, titulo: str):
    for cx, cy, w, h in res["caixas"]:
        rect = patches.Rectangle(
            (cx - w / 2.0, cy - h / 2.0), w, h, facecolor="#607d8b", edgecolor="black", alpha=0.85
        )
        ax.add_patch(rect)
    for cx, cy, r in res["circulos"]:
        circ = patches.Circle((cx, cy), r, facecolor="#607d8b", edgecolor="black", alpha=0.85)
        ax.add_patch(circ)

    # Desenha a Linha-m imaginária ligando S a T
    sx, sy = res["inicio"][0], res["inicio"][1]
    gx, gy = res["goal"]
    ax.plot([sx, gx], [sy, gy], "k--", lw=1.1, alpha=0.6, label="Linha-m")

    traj = res["traj"]
    ax.plot(traj[:, 0], traj[:, 1], "g-", lw=2.0, label="Trajetória Bug2")
    ax.scatter([sx], [sy], c="green", s=60, zorder=5, label="Partida (S)")
    ax.scatter([gx], [gy], c="red", marker="*", s=160, zorder=5, label="Meta (T)")

    if res["hit_points"]:
        hp = np.array(res["hit_points"])
        ax.scatter(hp[:, 0], hp[:, 1], c="darkorange", marker="^", s=85, zorder=6, label="Impacto (H_i)")
    if res["leave_points"]:
        lp = np.array(res["leave_points"])
        ax.scatter(lp[:, 0], lp[:, 1], c="magenta", marker="D", s=70, zorder=6, label="Saída (L_i)")

    ax.set_title(f"{titulo}\n({res['status']} | L={res['comprimento']:.2f} m)", fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7.2, loc="upper left")


def main():
    # Mesmos 3 cenários exatos do Passo 2.2 para comparação justa!
    res_a = rodar_cenario_bug2(
        nome="Cenário A — Obstáculos Esparsos e Corredor (Bug2)",
        inicio=(0.0, 0.0, 0.0),
        goal=(8.0, 0.0),
        caixas=[
            (2.2,  0.35, 0.8, 0.8),
            (4.3, -0.50, 0.8, 0.8),
            (6.0,  1.05, 1.4, 0.5),
            (6.0, -1.05, 1.4, 0.5),
        ],
        circulos=[(3.3, -0.25, 0.35)],
    )

    res_b = rodar_cenario_bug2(
        nome="Cenário B — Armadilha em 'U' (Bug2)",
        inicio=(0.0, 0.0, 0.0),
        goal=(6.0, 0.0),
        caixas=[
            (3.5,  0.00, 0.25, 2.2),
            (2.6,  1.00, 1.80, 0.25),
            (2.6, -1.00, 1.80, 0.25),
        ],
        circulos=[],
    )

    res_gnron = rodar_cenario_bug2(
        nome="Cenário GNRON — Parede atrás da meta (Bug2)",
        inicio=(0.0, 0.0, 0.0),
        goal=(6.0, 0.0),
        caixas=[
            (6.55, 0.0, 0.20, 2.0),
        ],
        circulos=[],
    )

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8))
    desenhar_cenario_bug2(axes[0], res_a, "1. Cenário A: Esparsos + Corredor")
    desenhar_cenario_bug2(axes[1], res_b, "2. Cenário B: Escape da Armadilha em 'U'")
    desenhar_cenario_bug2(axes[2], res_gnron, "3. Cenário GNRON: Imune à parede atrás")

    plt.tight_layout()
    saida = Path(__file__).parent / "resultado_passo3_bug2.png"
    plt.savefig(saida, dpi=150)
    print(f"\nGráfico salvo em: {saida}")

    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
