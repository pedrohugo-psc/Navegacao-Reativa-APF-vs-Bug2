"""
Fase 6 — Orquestrador Científico da Etapa II (Seções 4.6 e 5.1 do Edital).

Executa em modo headless no Gazebo Harmonic:
  1) Bateria Principal de Repetibilidade (N = 5 sementes fixas: 42, 43, 44, 45, 46):
       - Cenário A (Esparsos e Corredor): APF (5x) e Bug2 (5x)
       - Cenário B (Armadilha em "U"):    APF (5x) e Bug2 (5x)
       - Subcenário GNRON:                APF (5x) e Bug2 (5x)
  2) Estudo Paramétrico do APF (seed = 42):
       - Cenário A variando (rho_0, k_rep): fraco, nominal, forte e ganho alto
       - Cenário GNRON variando rho_0 em {0.35, 0.80, 1.10, 1.50} m

É resumível: se o JSON de uma rodada já existir, pula automaticamente!
Ao final, chama scripts/plot_fase6_resultados.py para gerar a Tabela 1 e os gráficos.

Uso:
    python3 scripts/run_fase6_experimentos.py
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


PROJETO_DIR = Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
BATERIA_DIR = PROJETO_DIR / "resultados" / "etapa2_bateria"
PARAM_DIR = PROJETO_DIR / "resultados" / "etapa2_parametrico"
ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(PROJETO_DIR / "ros2_ws" / "install" / "setup.bash")

SEEDS = [42, 43, 44, 45, 46]


def rodar_uma_execucao(
    cenario: str,
    metodo: str,
    csv_path: Path,
    seed: int = 42,
    k_att: float = 1.0,
    k_rep: float = 0.8,
    rho_0: float = 1.10,
    timeout_s: float = 75.0,
):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    json_path = csv_path.with_suffix(".json")
    if json_path.exists() and json_path.stat().st_size > 20:
        print(f"  [PULADO - já existe] {csv_path.name}", flush=True)
        return

    if csv_path.exists():
        csv_path.unlink()

    cmd = (
        f"source {ROS2_SETUP} && source {WS_SETUP} && "
        f"ros2 launch projeto6_navegacao etapa2_navegacao.launch.py "
        f"metodo:={metodo} cenario:={cenario} headless:=true "
        f"seed:={seed} k_att:={k_att} k_rep:={k_rep} rho_0:={rho_0} "
        f"output_csv:={csv_path}"
    )

    print(
        f">>> [{cenario}] {metodo.upper()} | seed={seed} | rho_0={rho_0} k_rep={k_rep} ...",
        flush=True,
    )
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    t0 = time.time()
    try:
        while time.time() - t0 < timeout_s:
            if json_path.exists() and json_path.stat().st_size > 20:
                time.sleep(0.5)
                break
            time.sleep(0.4)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        subprocess.run(["pkill", "-f", "gz sim"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    if json_path.exists():
        res = json.loads(json_path.read_text())
        print(
            f"    OK: {res['status']} | L={res['comprimento_m']:.2f} m | "
            f"T={res['tempo_s']:.1f} s | d_min={res['d_min_m']:.2f} m | col={res['colisoes']}",
            flush=True,
        )
    else:
        print(f"    [AVISO] Timeout de orquestrador ({timeout_s:.0f}s) em {csv_path.name}", flush=True)


def main():
    print("=========================================================================", flush=True)
    print(" FASE 6 — BATERIA PRINCIPAL (N = 5 SEMENTES: 42, 43, 44, 45, 46)", flush=True)
    print("=========================================================================", flush=True)

    bateria = [
        ("cenario_a", "apf", 65.0),
        ("cenario_a", "bug2", 75.0),
        ("cenario_b", "apf", 35.0),
        ("cenario_b", "bug2", 85.0),
        ("cenario_gnron", "apf", 40.0),
        ("cenario_gnron", "bug2", 40.0),
    ]

    for cenario, metodo, tout in bateria:
        for s in SEEDS:
            csv_path = BATERIA_DIR / f"{cenario}_{metodo}_seed{s}.csv"
            rodar_uma_execucao(cenario, metodo, csv_path, seed=s, timeout_s=tout)

    print("\n=========================================================================", flush=True)
    print(" FASE 6 — ESTUDO PARAMÉTRICO DO APF (Seção 4.6 do Edital)", flush=True)
    print("=========================================================================", flush=True)

    # Parte A: Sensibilidade a (rho_0, k_rep) no Cenário A
    configs_a = [
        ("fraco", 0.60, 0.40),
        ("nominal", 1.10, 0.80),
        ("forte", 1.60, 1.50),
        ("ganho_alto", 1.10, 2.20),
    ]
    for label, rho0, krep in configs_a:
        csv_path = PARAM_DIR / f"param_cenario_a_{label}.csv"
        rodar_uma_execucao(
            "cenario_a", "apf", csv_path, seed=42, k_att=1.0, k_rep=krep, rho_0=rho0, timeout_s=65.0
        )

    # Parte B: Limiar de GNRON variando rho_0 (parede a 0.45 m atrás da meta)
    for rho0 in (0.35, 0.80, 1.10, 1.50):
        label = f"rho{int(round(rho0 * 100)):03d}"
        csv_path = PARAM_DIR / f"param_gnron_{label}.csv"
        rodar_uma_execucao(
            "cenario_gnron", "apf", csv_path, seed=42, k_att=1.0, k_rep=0.80, rho_0=rho0, timeout_s=45.0
        )

    print("\n>>> Consolidando Tabela 1 e gerando figuras finais da Etapa II...", flush=True)
    subprocess.run([sys.executable, str(PROJETO_DIR / "scripts" / "plot_fase6_resultados.py")], check=True)


if __name__ == "__main__":
    main()
