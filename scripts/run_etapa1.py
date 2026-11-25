"""
Executa automaticamente os 3 ensaios obrigatórios da Etapa I no Gazebo Harmonic (headless)
e gera os gráficos e a tabela de RMSE ao final.

Uso:
    python3 scripts/run_etapa1.py
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


PROJETO_DIR = Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
RESULTADOS_DIR = PROJETO_DIR / "resultados" / "etapa1"
ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(PROJETO_DIR / "ros2_ws" / "install" / "setup.bash")

# (nome_trajetoria, tempo_max_espera_segundos)
ENSAIOS = [
    ("reta", 32.0),
    ("arco", 38.0),
    ("composto", 32.0),
]


def rodar_ensaio(trajetoria: str, timeout_s: float, noise_std: float = 0.05, seed: int = 42):
    RESULTADOS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = RESULTADOS_DIR / f"etapa1_{trajetoria}.csv"
    if csv_path.exists():
        csv_path.unlink()

    cmd = (
        f"source {ROS2_SETUP} && source {WS_SETUP} && "
        f"ros2 launch projeto6_navegacao etapa1_odometria.launch.py "
        f"trajetoria:={trajetoria} headless:=true "
        f"encoder_noise_std:={noise_std} seed:={seed} output_csv:={csv_path}"
    )

    print(f"\n>>> Executando Etapa I — Trajetória '{trajetoria}' (sigma={noise_std}, seed={seed})...")
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
            if csv_path.exists() and csv_path.stat().st_size > 100:
                # Aguarda 0.5 s para garantir flush completo do arquivo CSV
                time.sleep(0.5)
                break
            time.sleep(0.3)
    finally:
        # Encerra todo o grupo de processos (launch + gz sim + bridge + nós)
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

    if csv_path.exists():
        print(f"    OK! CSV salvo: {csv_path} ({csv_path.stat().st_size} bytes)")
    else:
        print(f"    [ERRO] CSV não foi gerado para '{trajetoria}' dentro de {timeout_s}s.")


def main():
    for traj, timeout_s in ENSAIOS:
        rodar_ensaio(traj, timeout_s)

    print("\n>>> Gerando gráficos e tabela resumo da Etapa I...")
    subprocess.run([sys.executable, str(PROJETO_DIR / "scripts" / "plot_etapa1.py")], check=True)


if __name__ == "__main__":
    main()
