"""
Fase 6 — Consolidação Estatística (Tabela 1 em Markdown + LaTeX) e Figuras Finais da Etapa II.

Gera:
  - resultados/etapa2_bateria/tabela1_metricas.md
  - resultados/etapa2_bateria/tabela1_latex.tex  (usando table* largura total, sem sobreposição!)
  - resultados/etapa2_bateria/fig3_etapa2_comparacao_cenarios.png
  - resultados/etapa2_bateria/fig4_etapa2_estudo_parametrico.png

Uso:
    python3 scripts/plot_fase6_resultados.py
    python3 scripts/plot_fase6_resultados.py --show
"""

import csv
import json
import sys
from pathlib import Path

import matplotlib
import numpy as np

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt


PROJETO_DIR = Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
BATERIA_DIR = PROJETO_DIR / "resultados" / "etapa2_bateria"
PARAM_DIR = PROJETO_DIR / "resultados" / "etapa2_parametrico"
SEEDS = [42, 43, 44, 45, 46]


# Geometrias exatas dos arquivos .sdf para desenho fiel nos mapas 2D
GEOMETRIAS = {
    "cenario_a": {
        "titulo": "Cenário A: Obstáculos Esparsos e Corredor",
        "goal": (8.0, 0.0),
        "caixas": [
            (2.2, 0.35, 0.8, 0.8),
            (4.3, -0.50, 0.8, 0.8),
            (6.0, 1.05, 1.4, 0.5),
            (6.0, -1.05, 1.4, 0.5),
        ],
        "circulos": [(3.3, -0.25, 0.35)],
    },
    "cenario_b": {
        "titulo": "Cenário B: Armadilha Côncava em 'U'",
        "goal": (6.0, 0.0),
        "caixas": [
            (3.5, 0.00, 0.25, 2.25),
            (2.6, 1.00, 1.80, 0.25),
            (2.6, -1.00, 1.80, 0.25),
        ],
        "circulos": [],
    },
    "cenario_gnron": {
        "titulo": "Subcenário GNRON (Parede a 0,45 m atrás da meta)",
        "goal": (6.0, 0.0),
        "caixas": [(6.55, 0.0, 0.20, 2.2)],
        "circulos": [],
    },
}


def carregar_csv(path: Path) -> dict[str, np.ndarray]:
    cols: dict[str, list[float]] = {}
    with open(path, "r", newline="") as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                cols.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in cols.items()}


def desenhar_obstaculos(ax, cenario: str):
    geo = GEOMETRIAS[cenario]
    for cx, cy, w, h in geo["caixas"]:
        ax.add_patch(
            patches.Rectangle(
                (cx - w / 2.0, cy - h / 2.0), w, h, facecolor="#607d8b", edgecolor="black", alpha=0.85
            )
        )
    for cx, cy, r in geo["circulos"]:
        ax.add_patch(patches.Circle((cx, cy), r, facecolor="#8d6e63", edgecolor="black", alpha=0.85))
    gx, gy = geo["goal"]
    ax.plot([0.0, gx], [0.0, gy], "k:", lw=1.0, alpha=0.55, label="Linha-m")
    ax.scatter([0.0], [0.0], c="green", s=60, zorder=6, label="Partida (S)")
    ax.scatter([gx], [gy], c="red", marker="*", s=170, zorder=6, label="Meta (T)")


def fmt_ms(vals: list[float], casas: int = 2) -> str:
    arr = np.array(vals, dtype=float)
    return f"{np.mean(arr):.{casas}f} ± {np.std(arr, ddof=0):.{casas}f}"


def fmt_ms_tex(vals: list[float], casas: int = 2) -> str:
    arr = np.array(vals, dtype=float)
    return f"${np.mean(arr):.{casas}f} \\pm {np.std(arr, ddof=0):.{casas}f}$"


def gerar_tabela1():
    linhas_md = [
        "# Tabela 1 — Comparação Quantitativa APF vs. Bug2 ($N = 5$ sementes: 42–46)\n",
        "| Cenário | Método | Sucesso (%) | Colisões | $L_{traj}$ (m) | Tempo (s) | $d_{min}$ (m) | Bloqueios (%) | Erro Odom $e_p(t_f)$ (m) | Dist. Real Meta (m) |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    linhas_tex = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Comparação Quantitativa entre APF e Bug2 nos Cenários Experimentais ($N = 5$ execuções, sementes 42--46, média $\pm$ desvio padrão).}",
        r"\label{tab:comparacao_p6}",
        r"\setlength{\tabcolsep}{5pt}",
        r"\begin{tabular}{llcccccccc}",
        r"\hline",
        r"\textbf{Cenário} & \textbf{Método} & \textbf{Sucesso (\%)} & \textbf{Colisões} & \textbf{$L_{\text{traj}}$ (m)} & \textbf{$T_{\text{tot}}$ (s)} & \textbf{$d_{\min}$ (m)} & \textbf{Bloqueios (\%)} & \textbf{$e_p(t_f)$ (m)} & \textbf{$d_{\text{meta}}$ (m)} \\",
        r"\hline",
    ]

    pares = [
        ("cenario_a", "Cenário A (Esparsos)", "apf", "APF"),
        ("cenario_a", "Cenário A (Esparsos)", "bug2", "Bug2"),
        ("cenario_b", "Cenário B (Armadilha ``U'')", "apf", "APF"),
        ("cenario_b", "Cenário B (Armadilha ``U'')", "bug2", "Bug2"),
        ("cenario_gnron", "Subcenário GNRON", "apf", "APF"),
        ("cenario_gnron", "Subcenário GNRON", "bug2", "Bug2"),
    ]

    print("\n=== TABELA 1 CONSOLIDADA (N = 5 SEMENTES: 42..46) ===")
    for cen, cen_nome, met, met_nome in pares:
        resumos = []
        for s in SEEDS:
            jp = BATERIA_DIR / f"{cen}_{met}_seed{s}.json"
            if jp.exists():
                resumos.append(json.loads(jp.read_text()))
        if not resumos:
            continue

        taxa_suc = 100.0 * sum(1 for r in resumos if r["sucesso"]) / len(resumos)
        taxa_bloq = 100.0 * sum(1 for r in resumos if r["bloqueio_minimo_local"]) / len(resumos)
        colisoes = sum(r["colisoes"] for r in resumos)
        l_vals = [r["comprimento_m"] for r in resumos]
        t_vals = [r["tempo_s"] for r in resumos]
        d_vals = [r["d_min_m"] for r in resumos]
        ep_vals = [r["erro_final_odom_m"] for r in resumos]
        dm_vals = [r["dist_final_meta_gt_m"] for r in resumos]

        cen_md = cen_nome.replace("``", '"').replace("''", '"')
        linha_md = (
            f"| {cen_md} | **{met_nome}** | {taxa_suc:.0f}% | {colisoes} | "
            f"{fmt_ms(l_vals)} | {fmt_ms(t_vals, 1)} | {fmt_ms(d_vals)} | "
            f"{taxa_bloq:.0f}% | {fmt_ms(ep_vals, 4)} | {fmt_ms(dm_vals)} |"
        )
        linhas_md.append(linha_md)
        print(linha_md)

        linhas_tex.append(
            f"{cen_nome} & {met_nome} & {taxa_suc:.0f}\\% & {colisoes} & "
            f"{fmt_ms_tex(l_vals)} & {fmt_ms_tex(t_vals, 1)} & {fmt_ms_tex(d_vals)} & "
            f"{taxa_bloq:.0f}\\% & {fmt_ms_tex(ep_vals, 4)} & {fmt_ms_tex(dm_vals)} \\\\"
        )

    linhas_tex.extend([r"\hline", r"\end{tabular}", r"\end{table*}"])

    md_path = BATERIA_DIR / "tabela1_metricas.md"
    tex_path = BATERIA_DIR / "tabela1_latex.tex"
    md_path.write_text("\n".join(linhas_md) + "\n")
    tex_path.write_text("\n".join(linhas_tex) + "\n")
    print(f"\nTabelas salvas em:\n  - {md_path}\n  - {tex_path}")


def gerar_figura3_comparacao():
    fig, axes = plt.subplots(1, 3, figsize=(16.0, 5.0))
    for ax, cen in zip(axes, ("cenario_a", "cenario_b", "cenario_gnron")):
        desenhar_obstaculos(ax, cen)

        for idx, s in enumerate(SEEDS):
            p_apf = BATERIA_DIR / f"{cen}_apf_seed{s}.csv"
            if p_apf.exists():
                d = carregar_csv(p_apf)
                ax.plot(
                    d["x_gt"],
                    d["y_gt"],
                    color="#1565c0",
                    lw=1.6,
                    alpha=0.70,
                    label="APF (5 sementes)" if idx == 0 else None,
                )
                if idx == 0:
                    j_apf = json.loads(p_apf.with_suffix(".json").read_text())
                    if not j_apf["sucesso"]:
                        ax.scatter(
                            [d["x_gt"][-1]],
                            [d["y_gt"][-1]],
                            c="orange",
                            marker="X",
                            s=110,
                            zorder=7,
                            label="Estagnação APF",
                        )

            p_bug = BATERIA_DIR / f"{cen}_bug2_seed{s}.csv"
            if p_bug.exists():
                d = carregar_csv(p_bug)
                ax.plot(
                    d["x_gt"],
                    d["y_gt"],
                    color="#2e7d32",
                    lw=1.6,
                    alpha=0.75,
                    label="Bug2 (5 sementes)" if idx == 0 else None,
                )
                if idx == 0:
                    j_bug = json.loads(p_bug.with_suffix(".json").read_text())
                    if j_bug.get("hit_points"):
                        hp = np.array(j_bug["hit_points"])
                        ax.scatter(
                            hp[:, 0], hp[:, 1], c="darkorange", marker="^", s=85, zorder=7, label="Hit ($H_i$)"
                        )
                    if j_bug.get("leave_points"):
                        lp = np.array(j_bug["leave_points"])
                        ax.scatter(
                            lp[:, 0], lp[:, 1], c="magenta", marker="D", s=70, zorder=7, label="Leave ($L_i$)"
                        )

        ax.set_title(GEOMETRIAS[cen]["titulo"], fontsize=10.5)
        ax.set_xlabel("Posição Global x (m)")
        ax.set_ylabel("Posição Global y (m)")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7.5, loc="upper left")

    plt.tight_layout()
    out = BATERIA_DIR / "fig3_etapa2_comparacao_cenarios.png"
    fig.savefig(out, dpi=220, bbox_inches="tight")
    print(f"Figura 3 salva em: {out}")


def gerar_figura4_parametrico():
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 4.9))

    # Painel Esquerdo: Sensibilidade a (rho_0, k_rep) no Cenário A
    ax = axes[0]
    desenhar_obstaculos(ax, "cenario_a")
    configs = [
        ("fraco", r"Fraco ($\rho_0=0.6,\; k_{rep}=0.4$)", "#00897b", "--"),
        ("nominal", r"Nominal ($\rho_0=1.1,\; k_{rep}=0.8$)", "#1565c0", "-"),
        ("forte", r"Forte ($\rho_0=1.6,\; k_{rep}=1.5$)", "#6a1b9a", "-."),
        ("ganho_alto", r"Ganho alto ($\rho_0=1.1,\; k_{rep}=2.2$)", "#c62828", ":"),
    ]
    for chave, rotulo, cor, estilo in configs:
        p = PARAM_DIR / f"param_cenario_a_{chave}.csv"
        if p.exists():
            d = carregar_csv(p)
            j = json.loads(p.with_suffix(".json").read_text())
            ax.plot(
                d["x_gt"],
                d["y_gt"],
                color=cor,
                linestyle=estilo,
                lw=2.0,
                label=f"{rotulo} [{j['status']}, $d_{{min}}$={j['d_min_m']:.2f}m]",
            )
    ax.set_title("Estudo Paramétrico do APF — Cenário A", fontsize=10.5)
    ax.set_xlabel("Posição Global x (m)")
    ax.set_ylabel("Posição Global y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7.6, loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=1)

    # Painel Direito: Limiar GNRON em função de rho_0
    ax2 = axes[1]
    rhos = [0.35, 0.80, 1.10, 1.50]
    dists_res = []
    status_list = []
    for r0 in rhos:
        p = PARAM_DIR / f"param_gnron_rho{int(round(r0 * 100)):03d}.json"
        if p.exists():
            j = json.loads(p.read_text())
            dists_res.append(j["dist_final_meta_gt_m"])
            status_list.append(j["status"])
        else:
            dists_res.append(np.nan)
            status_list.append("N/A")

    cores_bar = ["#2e7d32" if s == "SUCCESS" else "#d84315" for s in status_list]
    bars = ax2.bar([f"{r:.2f}" for r in rhos], dists_res, color=cores_bar, edgecolor="black", width=0.52)
    ax2.axhline(0.20, color="black", linestyle="--", lw=1.2, label="Tolerância de chegada ($d_{tol}=0,20$ m)")
    for bar, val, st in zip(bars, dists_res, status_list):
        if np.isfinite(val):
            ax2.text(
                bar.get_x() + bar.get_width() / 2.0,
                val + 0.025,
                f"{val:.2f} m\n({'OK' if st == 'SUCCESS' else 'GNRON'})",
                ha="center",
                va="bottom",
                fontsize=8.5,
                fontweight="bold",
            )
    ax2.set_title("Efeito de $\\rho_0$ no Fenômeno GNRON (parede a $0,45$ m da meta)", fontsize=10.5)
    ax2.set_xlabel("Raio de Influência Repulsiva $\\rho_0$ (m)")
    ax2.set_ylabel("Distância Residual Real até a Meta (m)")
    ax2.set_ylim(0, max([v for v in dists_res if np.isfinite(v)] + [0.6]) + 0.22)
    ax2.grid(True, axis="y", alpha=0.3)
    ax2.legend(fontsize=8.5, loc="upper left")

    plt.tight_layout()
    out = BATERIA_DIR / "fig4_etapa2_estudo_parametrico.png"
    fig.savefig(out, dpi=220, bbox_inches="tight")
    print(f"Figura 4 salva em: {out}")


def main():
    gerar_tabela1()
    gerar_figura3_comparacao()
    gerar_figura4_parametrico()
    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
