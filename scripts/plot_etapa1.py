"""
Script de Análise e Plotagem da Etapa I (Odometria RK2 vs. Ground Truth).

Lê os 3 CSVs gerados pelo metrics_logger_node em resultados/etapa1/:
  - etapa1_reta.csv
  - etapa1_arco.csv
  - etapa1_composto.csv

Gera as figuras prontas para análise e para o artigo IEEE (Seção IV-A):
  - fig1_etapa1_trajetorias.png
  - fig2_etapa1_erros.png

Uso:
    python3 scripts/plot_etapa1.py
    python3 scripts/plot_etapa1.py --show
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


PASTA_ETAPA1 = (
    Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
    / "resultados"
    / "etapa1"
)

ENSAIOS = [
    ("reta", "Trajetória 1: Linha Reta (5 m)"),
    ("arco", "Trajetória 2: Arco Circular (Rc = 1 m)"),
    ("composto", "Trajetória 3: Percurso Composto em 'S'"),
]


def carregar_csv(caminho: Path) -> dict[str, np.ndarray]:
    colunas: dict[str, list[float]] = {}
    with open(caminho, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                colunas.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in colunas.items()}


def main():
    dados = {}
    print("=== TABELA RESUMO DA ETAPA I (ODOMETRIA RK2 vs. GROUND TRUTH) ===")
    print(f"{'Trajetória':<38} | {'Comp.(m)':>8} | {'RMSE_p(m)':>9} | {'e_p(tf)':>8} | {'RMSE_th(°)':>10} | {'e_th(tf)(°)':>11}")
    print("-" * 98)

    for chave, titulo in ENSAIOS:
        arq = PASTA_ETAPA1 / f"etapa1_{chave}.csv"
        if not arq.exists():
            print(f"[AVISO] Arquivo não encontrado: {arq}")
            continue
        d = carregar_csv(arq)
        dados[chave] = (titulo, d)

        diffs = np.hypot(np.diff(d["x_gt"]), np.diff(d["y_gt"]))
        comp = float(np.sum(diffs))
        rmse_p = math.sqrt(float(np.mean(d["e_p"] ** 2)))
        ep_f = float(d["e_p"][-1])
        rmse_th = math.degrees(math.sqrt(float(np.mean(d["e_theta"] ** 2))))
        eth_f = math.degrees(float(d["e_theta"][-1]))
        print(f"{titulo:<38} | {comp:8.2f} | {rmse_p:9.4f} | {ep_f:8.4f} | {rmse_th:10.2f} | {eth_f:11.2f}")

    if not dados:
        print("Nenhum CSV da Etapa I encontrado. Rode primeiro os ensaios no Gazebo.")
        return

    # Figura 1: Trajetórias sobrepostas (Ground Truth vs Odometria RK2)
    fig1, axes1 = plt.subplots(1, len(dados), figsize=(5.2 * len(dados), 4.5))
    if len(dados) == 1:
        axes1 = [axes1]

    for ax, (chave, (titulo, d)) in zip(axes1, dados.items()):
        ax.plot(d["x_gt"], d["y_gt"], "k-", lw=2.2, label="Ground Truth (Gazebo)")
        ax.plot(d["x_est"], d["y_est"], "b--", lw=1.8, label="Odometria RK2 (Encoders)")
        ax.scatter([d["x_gt"][0]], [d["y_gt"][0]], c="green", s=55, zorder=5, label="Início")
        ax.set_title(titulo, fontsize=10)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    plt.tight_layout()
    f1_path = PASTA_ETAPA1 / "fig1_etapa1_trajetorias.png"
    fig1.savefig(f1_path, dpi=200)

    # Figura 2: Evolução temporal dos erros e_p(t) e e_theta(t)
    fig2, axes2 = plt.subplots(1, len(dados), figsize=(5.2 * len(dados), 4.2))
    if len(dados) == 1:
        axes2 = [axes2]

    for ax, (chave, (titulo, d)) in zip(axes2, dados.items()):
        ax2 = ax.twinx()
        l1 = ax.plot(d["t"], d["e_p"], "b-", lw=1.8, label="Erro Posição e_p(t) [m]")
        l2 = ax2.plot(d["t"], np.degrees(d["e_theta"]), "r--", lw=1.5, label="Erro Angular e_theta(t) [°]")
        ax.set_title(titulo, fontsize=10)
        ax.set_xlabel("Tempo de Simulação (s)")
        ax.set_ylabel("e_p (m)", color="b")
        ax2.set_ylabel("e_theta (graus)", color="r")
        ax.grid(True, alpha=0.3)
        linhas = l1 + l2
        ax.legend(linhas, [l.get_label() for l in linhas], loc="upper left", fontsize=8)

    plt.tight_layout()
    f2_path = PASTA_ETAPA1 / "fig2_etapa1_erros.png"
    fig2.savefig(f2_path, dpi=200)

    print(f"\nFiguras salvas em:\n  - {f1_path}\n  - {f2_path}")
    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
