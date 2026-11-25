"""
Modelo Cinemático Diferencial e Utilitários Geométricos em SE(2).
Mesmas equações validadas na Fase 1 (teoria) e Fase 2 (Python puro).
"""

import math


def wrap_to_pi(angulo: float) -> float:
    """Normaliza qualquer ângulo para o intervalo (-pi, pi] rad."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Converte ângulo planar yaw (theta) em quatérnio (qx, qy, qz, qw) do ROS 2."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extrai o ângulo planar yaw (theta) de um quatérnio (qx, qy, qz, qw) do ROS 2."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def cinematica_direta(w_d: float, w_e: float, r: float, l: float) -> tuple[float, float]:
    """
    Mapeamento cinemático direto das rodas para o chassi (ponto médio P):
        v = (R / 2) * (w_d + w_e)   [m/s]
        w = (R / L) * (w_d - w_e)   [rad/s]
    """
    v = 0.5 * r * (w_d + w_e)
    w = (r / l) * (w_d - w_e)
    return v, w


def passo_odometria_rk2(
    x: float,
    y: float,
    theta: float,
    w_d: float,
    w_e: float,
    dt: float,
    r: float,
    l: float,
) -> tuple[float, float, float, float, float]:
    """
    Passo discreto de integração odométrica pelo Método do Ponto Médio (RK2).
    Retorna: (x_novo, y_novo, theta_novo, v, w)
    """
    v, w = cinematica_direta(w_d, w_e, r, l)
    ds = v * dt
    dtheta = w * dt

    theta_medio = theta + 0.5 * dtheta
    x_novo = x + ds * math.cos(theta_medio)
    y_novo = y + ds * math.sin(theta_medio)
    theta_novo = wrap_to_pi(theta + dtheta)
    return x_novo, y_novo, theta_novo, v, w
