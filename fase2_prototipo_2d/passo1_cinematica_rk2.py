"""
Fase 2 — Passo 2.1: Laboratório Matemático 2D (Sem ROS e Sem Gazebo)
Cinemática Direta Diferencial, Integração Euler vs. RK2 (Ponto Médio) e Deriva Odométrica.

Projeto 6 — PPGEE0060 (Robótica Móvel / UFBA)
Execute com:
    python3 passo1_cinematica_rk2.py
Para abrir a janela interativa do gráfico (além de salvar o PNG):
    python3 passo1_cinematica_rk2.py --show
"""

import math
import random
import sys
from pathlib import Path

import matplotlib
import numpy as np

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ==============================================================================
# 1. PARÂMETROS FÍSICOS DO ROBÔ DIFERENCIAL (Decisão D2 do memoria.md)
# ==============================================================================
R = 0.05   # Raio das rodas motrizes [m]
L = 0.35   # Bitola (distância entre os centros das rodas esquerda e direita) [m]


# ==============================================================================
# 2. FUNÇÕES MATEMÁTICAS PURAS DA ETAPA I
#    (Estas mesmas equações irão para o nó ROS 2 na Fase 4 e para o STM32 em C!)
# ==============================================================================
def wrap_to_pi(angulo: float) -> float:
    """Mantém qualquer ângulo normalizado no intervalo (-pi, pi] rad."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def cinematica_direta(w_d: float, w_e: float, r: float = R, l: float = L) -> tuple[float, float]:
    """
    Converte velocidades angulares das rodas [rad/s] em velocidades do chassi:
        v: velocidade linear do ponto médio P [m/s]
        w: velocidade angular de guinada (yaw rate) [rad/s]
           (w > 0 vira para a ESQUERDA; w < 0 vira para a DIREITA)
    """
    v = (r / 2.0) * (w_d + w_e)
    w = (r / l) * (w_d - w_e)
    return v, w


def cinematica_inversa(v: float, w: float, r: float = R, l: float = L) -> tuple[float, float]:
    """
    Dado um comando desejado (v, w) para o robô, calcula qual velocidade
    cada roda (w_d, w_e) precisa girar para produzir esse movimento.
    """
    w_d = (v + (l / 2.0) * w) / r
    w_e = (v - (l / 2.0) * w) / r
    return w_d, w_e


def passo_odometria_euler(
    x: float, y: float, theta: float, w_d: float, w_e: float, dt: float, r: float = R, l: float = L
) -> tuple[float, float, float]:
    """Integração de 1ª ordem (Euler explícito): usa apenas o ângulo inicial theta."""
    v, w = cinematica_direta(w_d, w_e, r, l)
    ds = v * dt
    dtheta = w * dt

    x_novo = x + ds * math.cos(theta)
    y_novo = y + ds * math.sin(theta)
    theta_novo = wrap_to_pi(theta + dtheta)
    return x_novo, y_novo, theta_novo


def passo_odometria_rk2(
    x: float, y: float, theta: float, w_d: float, w_e: float, dt: float, r: float = R, l: float = L
) -> tuple[float, float, float]:
    """
    Integração de 2ª ordem (Método do Ponto Médio / RK2):
    projeta o deslocamento ds usando o ângulo médio (theta + dtheta / 2).
    """
    v, w = cinematica_direta(w_d, w_e, r, l)
    ds = v * dt
    dtheta = w * dt

    theta_medio = theta + 0.5 * dtheta
    x_novo = x + ds * math.cos(theta_medio)
    y_novo = y + ds * math.sin(theta_medio)
    theta_novo = wrap_to_pi(theta + dtheta)
    return x_novo, y_novo, theta_novo


# ==============================================================================
# 3. EXPERIMENTO A: EULER vs. RK2 EM UM CÍRCULO COMPLETO (Raio Rc = 1.0 m)
#    Usamos um dt grande (0.25 s) de propósito para enxergar o erro de truncamento!
# ==============================================================================
def experimento_euler_vs_rk2():
    dt = 0.25  # Passo grande [s] para evidenciar a diferença geométrica
    v_cmd = 0.25  # m/s
    w_cmd = 0.25  # rad/s -> raio Rc = v / w = 1.0 m
    tempo_volta = (2.0 * math.pi) / w_cmd  # ~25.13 s para dar 1 volta completa
    n_passos = int(round(tempo_volta / dt))

    w_d, w_e = cinematica_inversa(v_cmd, w_cmd)

    # Estados iniciais na origem (0, 0, 0)
    x_eu, y_eu, th_eu = 0.0, 0.0, 0.0
    x_rk, y_rk, th_rk = 0.0, 0.0, 0.0

    traj_eu = [(x_eu, y_eu)]
    traj_rk = [(x_rk, y_rk)]
    traj_exata = [(0.0, 0.0)]

    rc = v_cmd / w_cmd
    for k in range(1, n_passos + 1):
        t = k * dt
        # Solução analítica exata de um círculo de centro (0, Rc) e raio Rc
        x_ex = rc * math.sin(w_cmd * t)
        y_ex = rc * (1.0 - math.cos(w_cmd * t))
        traj_exata.append((x_ex, y_ex))

        x_eu, y_eu, th_eu = passo_odometria_euler(x_eu, y_eu, th_eu, w_d, w_e, dt)
        x_rk, y_rk, th_rk = passo_odometria_rk2(x_rk, y_rk, th_rk, w_d, w_e, dt)

        traj_eu.append((x_eu, y_eu))
        traj_rk.append((x_rk, y_rk))

    traj_exata = np.array(traj_exata)
    traj_eu = np.array(traj_eu)
    traj_rk = np.array(traj_rk)

    err_eu = np.linalg.norm(traj_exata - traj_eu, axis=1)
    err_rk = np.linalg.norm(traj_exata - traj_rk, axis=1)

    print("=== EXPERIMENTO A: Círculo Completo (Rc = 1.0 m, dt = 0.25 s, sem ruído) ===")
    print(f"  RMSE Posição Euler : {math.sqrt(np.mean(err_eu**2)):.4f} m | Erro final: {err_eu[-1]:.4f} m")
    print(f"  RMSE Posição RK2   : {math.sqrt(np.mean(err_rk**2)):.4f} m | Erro final: {err_rk[-1]:.4f} m")
    return traj_exata, traj_eu, traj_rk


# ==============================================================================
# 4. EXPERIMENTO B: PERCURSO EM "S" A 50 Hz COM RUÍDO NOS ENCODERS
#    Simula o efeito real que veremos na Etapa I dentro do Gazebo!
# ==============================================================================
def experimento_deriva_em_s(seed: int = 42, sigma_encoder: float = 0.15):
    rng = random.Random(seed)
    dt = 0.02  # 50 Hz (mesma taxa definida na Decisão D4 do memoria.md)

    # Perfil de velocidades (v [m/s], w [rad/s], duração [s]) formando um "S"
    segmentos = [
        (0.25,  0.00, 4.0),   # Trecho 1: 1.0 m em linha reta
        (0.25, +0.35, 4.5),   # Trecho 2: curva para a esquerda (~90 graus)
        (0.25,  0.00, 3.0),   # Trecho 3: reta intermediária
        (0.25, -0.35, 4.5),   # Trecho 4: curva para a direita (~90 graus)
        (0.25,  0.00, 4.0),   # Trecho 5: reta final
    ]

    # Ground Truth (movimento real sem ruído) vs. Odometria estimada (com ruído nos encoders)
    x_gt, y_gt, th_gt = 0.0, 0.0, 0.0
    x_est, y_est, th_est = 0.0, 0.0, 0.0

    tempos = [0.0]
    gt_hist = [(x_gt, y_gt, th_gt)]
    est_hist = [(x_est, y_est, th_est)]

    t = 0.0
    for v_cmd, w_cmd, duracao in segmentos:
        passos = int(round(duracao / dt))
        w_d_real, w_e_real = cinematica_inversa(v_cmd, w_cmd)

        for _ in range(passos):
            t += dt
            # 1) Mundo real (Ground Truth) evolui com as velocidades reais das rodas
            x_gt, y_gt, th_gt = passo_odometria_rk2(x_gt, y_gt, th_gt, w_d_real, w_e_real, dt)

            # 2) Encoders medem com pequeno ruído gaussiano + leve viés sistemático (0.5%)
            w_d_medido = w_d_real * 1.005 + rng.gauss(0.0, sigma_encoder)
            w_e_medido = w_e_real + rng.gauss(0.0, sigma_encoder)

            # 3) Nosso estimador RK2 só enxerga os encoders ruidosos!
            x_est, y_est, th_est = passo_odometria_rk2(x_est, y_est, th_est, w_d_medido, w_e_medido, dt)

            tempos.append(t)
            gt_hist.append((x_gt, y_gt, th_gt))
            est_hist.append((x_est, y_est, th_est))

    tempos = np.array(tempos)
    gt_hist = np.array(gt_hist)
    est_hist = np.array(est_hist)

    # Métricas exigidas na Etapa I do edital
    e_p = np.sqrt((gt_hist[:, 0] - est_hist[:, 0]) ** 2 + (gt_hist[:, 1] - est_hist[:, 1]) ** 2)
    e_theta = np.array([abs(wrap_to_pi(g - e)) for g, e in zip(gt_hist[:, 2], est_hist[:, 2])])

    rmse_p = math.sqrt(float(np.mean(e_p ** 2)))
    rmse_theta = math.sqrt(float(np.mean(e_theta ** 2)))

    print("\n=== EXPERIMENTO B: Percurso em 'S' a 50 Hz (seed=42, ruído nos encoders) ===")
    print(f"  RMSE Posição    (RMSE_p)     : {rmse_p:.4f} m  | Erro final e_p(tf): {e_p[-1]:.4f} m")
    print(f"  RMSE Orientação (RMSE_theta) : {math.degrees(rmse_theta):.2f} deg | Erro final e_th(tf): {math.degrees(e_theta[-1]):.2f} deg")
    return tempos, gt_hist, est_hist, e_p, e_theta


# ==============================================================================
# 5. PLOTAGEM DOS RESULTADOS
# ==============================================================================
def main():
    traj_exata, traj_eu, traj_rk = experimento_euler_vs_rk2()
    tempos, gt_hist, est_hist, e_p, e_theta = experimento_deriva_em_s(seed=42, sigma_encoder=0.15)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))

    # Painel 1: Euler vs RK2 no Círculo
    ax = axes[0]
    ax.plot(traj_exata[:, 0], traj_exata[:, 1], "k-", lw=2.0, label="Exata (Analítica)")
    ax.plot(traj_eu[:, 0], traj_eu[:, 1], "r--", lw=1.6, label="Euler (1ª ordem)")
    ax.plot(traj_rk[:, 0], traj_rk[:, 1], "b:", lw=2.2, label="RK2 (Ponto Médio)")
    ax.set_title("Exp. A: Euler vs. RK2 (dt = 0,25 s)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Painel 2: Percurso em "S" (Ground Truth vs Odometria RK2)
    ax = axes[1]
    ax.plot(gt_hist[:, 0], gt_hist[:, 1], "k-", lw=2.0, label="Ground Truth (Real)")
    ax.plot(est_hist[:, 0], est_hist[:, 1], "b--", lw=1.8, label="Odometria RK2 (Encoders)")
    ax.scatter([0], [0], c="green", s=50, zorder=5, label="Início (0,0)")
    ax.set_title("Exp. B: Trajetória em 'S' (50 Hz)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Painel 3: Evolução temporal dos erros e_p(t) e e_theta(t)
    ax = axes[2]
    ax2 = ax.twinx()
    l1 = ax.plot(tempos, e_p, "b-", lw=1.8, label="Erro de Posição e_p(t) [m]")
    l2 = ax2.plot(tempos, np.degrees(e_theta), "r--", lw=1.5, label="Erro Angular e_theta(t) [graus]")
    ax.set_title("Evolução do Erro Odométrico")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("e_p (m)", color="b")
    ax2.set_ylabel("e_theta (graus)", color="r")
    ax.grid(True, alpha=0.3)
    linhas = l1 + l2
    ax.legend(linhas, [l.get_label() for l in linhas], loc="upper left", fontsize=8)

    plt.tight_layout()
    saida = Path(__file__).parent / "resultado_passo1_odometria.png"
    plt.savefig(saida, dpi=150)
    print(f"\nGráfico salvo em: {saida}")

    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
