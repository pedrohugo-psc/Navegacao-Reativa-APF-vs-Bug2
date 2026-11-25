"""
Fase 2 — Passo 2.2: Laboratório Matemático 2D (Sem ROS e Sem Gazebo)
Campos Potenciais Artificiais (APF — Khatib, 1986) e suas Limitações (Koren & Borenstein, 1991).

Reutiliza a cinemática e odometria RK2 de passo1_cinematica_rk2.py!
Execute com:
    python3 passo2_apf_2d.py
Para abrir a janela interativa do gráfico (além de salvar o PNG):
    python3 passo2_apf_2d.py --show
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

# Reutilizamos exatamente as funções matemáticas construídas no Passo 2.1!
from passo1_cinematica_rk2 import (
    cinematica_inversa,
    passo_odometria_rk2,
    wrap_to_pi,
)


# ==============================================================================
# 1. SIMULADOR SIMPLES DE LiDAR 2D (Ray-Casting Geométrico)
#    No Gazebo, o plugin gpu_lidar fará exatamente isso em 3D e publicará em /scan.
# ==============================================================================
def retangulo_para_segmentos(cx: float, cy: float, largura: float, altura: float):
    """Converte um retângulo centrado em (cx, cy) nos seus 4 segmentos de parede."""
    hx, hy = largura / 2.0, altura / 2.0
    p1 = (cx - hx, cy - hy)
    p2 = (cx + hx, cy - hy)
    p3 = (cx + hx, cy + hy)
    p4 = (cx - hx, cy + hy)
    return [(p1, p2), (p2, p3), (p3, p4), (p4, p1)]


def intersecao_raio_segmento(ox: float, oy: float, dx: float, dy: float, a: tuple, b: tuple) -> float | None:
    """
    Calcula a distância t >= 0 do raio P(t) = (ox, oy) + t*(dx, dy) até o segmento AB.
    Retorna None se não houver interseção.
    """
    ax, ay = a
    bx, by = b
    sx, sy = bx - ax, by - ay
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-10:
        return None
    qpx, qpy = ax - ox, ay - oy
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * dy - qpy * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def simular_lidar_2d(
    x: float,
    y: float,
    theta: float,
    segmentos: list,
    circulos: list,
    n_feixes: int = 360,
    alcance_max: float = 8.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Retorna:
        angulos_rel: ângulos dos feixes no referencial do robô [-pi, pi)
        distancias:  menor distância medida por cada feixe (limitada a alcance_max)
    """
    angulos_rel = np.linspace(-math.pi, math.pi, n_feixes, endpoint=False)
    distancias = np.full(n_feixes, alcance_max, dtype=float)

    for i, alpha in enumerate(angulos_rel):
        ang_global = theta + alpha
        dx = math.cos(ang_global)
        dy = math.sin(ang_global)
        d_min = alcance_max

        for a, b in segmentos:
            t = intersecao_raio_segmento(x, y, dx, dy, a, b)
            if t is not None and t < d_min:
                d_min = t

        for cx, cy, raio in circulos:
            # Interseção raio-círculo: ||O + t*D - C||^2 = raio^2
            ocx, ocy = x - cx, y - cy
            b_quad = 2.0 * (ocx * dx + ocy * dy)
            c_quad = ocx * ocx + ocy * ocy - raio * raio
            disc = b_quad * b_quad - 4.0 * c_quad
            if disc >= 0.0:
                t_hit = (-b_quad - math.sqrt(disc)) / 2.0
                if 0.0 <= t_hit < d_min:
                    d_min = t_hit

        distancias[i] = d_min

    return angulos_rel, distancias


# ==============================================================================
# 2. CONTROLADOR APF (Equações do Módulo 1.2)
#    (Esta lógica irá para o nó apf_navigator_node.py na Fase 5!)
# ==============================================================================
def calcular_controle_apf(
    x: float,
    y: float,
    theta: float,
    goal: tuple[float, float],
    angulos_rel: np.ndarray,
    distancias: np.ndarray,
    k_att: float = 1.0,
    k_rep: float = 0.8,
    rho_0: float = 1.10,
    k_v: float = 0.25,
    k_w: float = 1.8,
    v_max: float = 0.40,
    w_max: float = 1.20,
    alcance_max: float = 8.0,
) -> tuple[float, float, np.ndarray, np.ndarray, np.ndarray]:
    """Calcula F_att, F_rep, F_total e converte em comandos diferenciais (v, w)."""
    # 1) Força Atrativa: F_att = k_att * (q_goal - q)
    f_att = np.array([
        k_att * (goal[0] - x),
        k_att * (goal[1] - y),
    ])

    # 2) Força Repulsiva somada sobre os feixes ativos do LiDAR (rho_i <= rho_0)
    f_rep = np.zeros(2, dtype=float)
    d_alpha = (2.0 * math.pi) / len(angulos_rel)
    rho_seguro = 0.12  # Evita divisão por zero se encostar muito perto

    for alpha, rho in zip(angulos_rel, distancias):
        if rho <= rho_0 and rho < alcance_max:
            rho_eff = max(rho, rho_seguro)
            mag = k_rep * ((1.0 / rho_eff) - (1.0 / rho_0)) * (1.0 / (rho_eff ** 2)) * d_alpha
            ang_global = theta + alpha
            # Empurra na direção OPOSTA ao feixe (-cos, -sin)
            f_rep[0] += mag * (-math.cos(ang_global))
            f_rep[1] += mag * (-math.sin(ang_global))

    # 3) Força Resultante Total
    f_total = f_att + f_rep
    norma_f = float(np.linalg.norm(f_total))

    if norma_f < 1e-6:
        return 0.0, 0.0, f_att, f_rep, f_total

    # 4) Conversão para comandos diferenciais (v, w) com frenagem em curvas fechadas
    theta_d = math.atan2(f_total[1], f_total[0])
    e_theta = wrap_to_pi(theta_d - theta)

    w_cmd = float(np.clip(k_w * e_theta, -w_max, w_max))
    # Fator max(0, cos(e_theta)): se precisar virar > 90 graus, freia v=0 e gira no eixo!
    fator_alinhamento = max(0.0, math.cos(e_theta))
    v_cmd = float(np.clip(k_v * norma_f * fator_alinhamento, 0.0, v_max))

    return v_cmd, w_cmd, f_att, f_rep, f_total


# ==============================================================================
# 3. SIMULAÇÃO DE UM CENÁRIO COMPLETO
# ==============================================================================
def rodar_cenario_apf(
    nome: str,
    inicio: tuple[float, float, float],
    goal: tuple[float, float],
    caixas: list[tuple[float, float, float, float]],
    circulos: list[tuple[float, float, float]],
    tempo_max: float = 45.0,
    rho_0: float = 1.10,
    k_att: float = 1.0,
    k_rep: float = 0.8,
) -> dict:
    dt = 0.05  # 20 Hz (mesma taxa do LiDAR no Gazebo)
    passos_max = int(tempo_max / dt)
    d_tol = 0.15  # Raio de chegada à meta [m]
    raio_robo = 0.20

    segmentos = []
    for cx, cy, w, h in caixas:
        segmentos.extend(retangulo_para_segmentos(cx, cy, w, h))

    x, y, theta = inicio
    traj = [(x, y, theta)]
    d_min_global = float("inf")
    status = "TIMEOUT"
    forca_final = np.zeros(2)

    # Janela deslizante de 5 segundos para detectar mínimo local / estagnação
    janela_passos = int(5.0 / dt)

    for k in range(passos_max):
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

        v_cmd, w_cmd, _, _, f_total = calcular_controle_apf(
            x, y, theta, goal, angulos, dists, k_att=k_att, k_rep=k_rep, rho_0=rho_0
        )
        forca_final = f_total

        # Integra movimento do robô diferencial usando nossas equações RK2 do Passo 2.1!
        w_d, w_e = cinematica_inversa(v_cmd, w_cmd)
        x, y, theta = passo_odometria_rk2(x, y, theta, w_d, w_e, dt)
        traj.append((x, y, theta))

        # Detector de mínimo local / GNRON: andou < 5 cm nos últimos 5 segundos
        if len(traj) > janela_passos:
            x_ant, y_ant, _ = traj[-janela_passos]
            desloc = math.hypot(x - x_ant, y - y_ant)
            if desloc < 0.05 and dist_goal > d_tol:
                status = "MINIMO_LOCAL_OU_GNRON"
                break

    traj_arr = np.array(traj)
    diffs = np.diff(traj_arr[:, :2], axis=0)
    comprimento = float(np.sum(np.linalg.norm(diffs, axis=1))) if len(diffs) > 0 else 0.0
    dist_final_meta = math.hypot(goal[0] - traj_arr[-1, 0], goal[1] - traj_arr[-1, 1])

    print(f"[{nome}]")
    print(
        f"  Status: {status} | Distância final até meta: {dist_final_meta:.3f} m | "
        f"Comprimento: {comprimento:.2f} m | d_min: {d_min_global:.2f} m | "
        f"||F_total|| final: {np.linalg.norm(forca_final):.3f}"
    )

    return {
        "nome": nome,
        "traj": traj_arr,
        "inicio": inicio,
        "goal": goal,
        "caixas": caixas,
        "circulos": circulos,
        "status": status,
        "dist_final": dist_final_meta,
    }


def desenhar_cenario(ax, res: dict, titulo: str):
    for cx, cy, w, h in res["caixas"]:
        rect = patches.Rectangle(
            (cx - w / 2.0, cy - h / 2.0), w, h, facecolor="#607d8b", edgecolor="black", alpha=0.85
        )
        ax.add_patch(rect)
    for cx, cy, r in res["circulos"]:
        circ = patches.Circle((cx, cy), r, facecolor="#607d8b", edgecolor="black", alpha=0.85)
        ax.add_patch(circ)

    traj = res["traj"]
    ax.plot(traj[:, 0], traj[:, 1], "b-", lw=2.0, label="Trajetória APF")
    ax.scatter([res["inicio"][0]], [res["inicio"][1]], c="green", s=60, zorder=5, label="Partida (S)")
    ax.scatter([res["goal"][0]], [res["goal"][1]], c="red", marker="*", s=160, zorder=5, label="Meta (T)")
    ax.scatter([traj[-1, 0]], [traj[-1, 1]], c="orange", marker="X", s=90, zorder=6, label="Parada final")

    ax.set_title(f"{titulo}\n({res['status']} | d_meta={res['dist_final']:.2f} m)", fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")


def main():
    # 1) Cenário A: Obstáculos esparsos e passagem de corredor (APF funciona muito bem!)
    res_a = rodar_cenario_apf(
        nome="Cenário A — Obstáculos Esparsos e Corredor",
        inicio=(0.0, 0.0, 0.0),
        goal=(8.0, 0.0),
        caixas=[
            (2.2,  0.35, 0.8, 0.8),
            (4.3, -0.50, 0.8, 0.8),
            (6.0,  1.05, 1.4, 0.5),  # Parede superior do corredor
            (6.0, -1.05, 1.4, 0.5),  # Parede inferior do corredor
        ],
        circulos=[(3.3, -0.25, 0.35)],
        rho_0=1.0,
        k_att=1.0,
        k_rep=0.7,
    )

    # 2) Cenário B: Armadilha Côncava em "U" (F_att e F_rep se cancelam -> F_total = 0!)
    res_b = rodar_cenario_apf(
        nome="Cenário B — Armadilha em 'U' (Mínimo Local)",
        inicio=(0.0, 0.0, 0.0),
        goal=(6.0, 0.0),
        caixas=[
            (3.5,  0.00, 0.25, 2.2),  # Fundo do "U"
            (2.6,  1.00, 1.80, 0.25), # Aba superior do "U"
            (2.6, -1.00, 1.80, 0.25), # Aba inferior do "U"
        ],
        circulos=[],
        rho_0=1.10,
        k_att=1.0,
        k_rep=0.8,
    )

    # 3) Cenário C: GNRON (Parede a 0.45 m ATRÁS da meta, dentro de rho_0 = 1.10 m)
    res_gnron = rodar_cenario_apf(
        nome="Cenário GNRON — Obstáculo Próximo à Meta",
        inicio=(0.0, 0.0, 0.0),
        goal=(6.0, 0.0),
        caixas=[
            (6.55, 0.0, 0.20, 2.0),   # Parede atrás da meta (borda frontal em x = 6.45 m -> d = 0.45 m < rho_0)
        ],
        circulos=[],
        rho_0=1.10,
        k_att=1.0,
        k_rep=0.8,
    )

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8))
    desenhar_cenario(axes[0], res_a, "1. Cenário A: Esparsos + Corredor")
    desenhar_cenario(axes[1], res_b, "2. Cenário B: Armadilha em 'U'")
    desenhar_cenario(axes[2], res_gnron, "3. Fenômeno GNRON (Parede atrás da meta)")

    plt.tight_layout()
    saida = Path(__file__).parent / "resultado_passo2_apf.png"
    plt.savefig(saida, dpi=150)
    print(f"\nGráfico salvo em: {saida}")

    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
