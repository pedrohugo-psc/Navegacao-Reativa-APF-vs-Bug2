"""
Validação funcional da Fase 5 — 1 rodada (seed=42) para cada combinação
essencial no Gazebo Harmonic (headless) antes da bateria estatística da Fase 6.
"""

import json
import os
import signal
import subprocess
import time
from pathlib import Path


PROJETO_DIR = Path("/mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo")
SAIDA_DIR = PROJETO_DIR / "resultados" / "fase5_validacao"
ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(PROJETO_DIR / "ros2_ws" / "install" / "setup.bash")

TESTES = [
    ("cenario_a", "apf", 55.0),
    ("cenario_a", "bug2", 65.0),
    ("cenario_b", "apf", 35.0),
    ("cenario_b", "bug2", 80.0),
    ("cenario_gnron", "apf", 40.0),
]


def rodar_teste(cenario: str, metodo: str, timeout_s: float):
    SAIDA_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = SAIDA_DIR / f"{cenario}_{metodo}_seed42.csv"
    json_path = csv_path.with_suffix(".json")
    for p in (csv_path, json_path):
        if p.exists():
            p.unlink()

    cmd = (
        f"source {ROS2_SETUP} && source {WS_SETUP} && "
        f"ros2 launch projeto6_navegacao etapa2_navegacao.launch.py "
        f"metodo:={metodo} cenario:={cenario} headless:=true seed:=42 "
        f"output_csv:={csv_path}"
    )

    print(f"\n>>> Rodando [{cenario}] com [{metodo.upper()}]...")
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
            f"    Status: {res['status']} | Colisões: {res['colisoes']} | "
            f"L_traj: {res['comprimento_m']:.2f} m | Tempo: {res['tempo_s']:.1f} s | "
            f"d_min: {res['d_min_m']:.2f} m | Dist. Meta: {res['dist_final_meta_gt_m']:.2f} m"
        )
        return res
    else:
        print(f"    [AVISO] Não finalizou dentro de {timeout_s:.0f}s.")
        return None


def main():
    for cen, met, tout in TESTES:
        rodar_teste(cen, met, tout)


if __name__ == "__main__":
    main()
