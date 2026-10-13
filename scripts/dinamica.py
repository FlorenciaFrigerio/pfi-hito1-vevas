#!/usr/bin/env python3
"""
Modelo dinamico y evaluacion de torque.

    tau = M(q) qdd + C(q, qd) qd + g(q)

Siguiendo el alcance fijado por el docente (handoff W09-S02), NO se
deriva simbolicamente la dinamica de los 6 GDL del IRB 120: el
fabricante no publica los tensores de inercia de los eslabones, de modo
que cualquier matriz M de 6x6 seria inventada. Se desarrolla en cambio
el brazo planar 2R formado por las juntas 2 y 3 (hombro y codo), que es
el subsistema que soporta el par gravitatorio dominante, y se alimenta
con q(t), qd(t), qdd(t) del perfil LSPB.

Parametros inerciales: estimacion declarada a partir de la masa total
del IRB 120 (25 kg) repartida entre brazo y antebrazo, con el centro de
masa al 50 % del eslabon y el eslabon modelado como barra delgada
(I = m L^2 / 12). Toda conclusion cuantitativa es por tanto un orden de
magnitud, no un valor certificado.

Grupo VEVAS
"""

import numpy as np

G = 9.81

# Eslabon 1 = brazo (a2), eslabon 2 = antebrazo (hasta la muneca)
L1, L2 = 0.270, 0.302
M1, M2 = 6.0, 4.0
LC1, LC2 = L1/2, L2/2
I1, I2 = M1*L1**2/12, M2*L2**2/12


def M_matriz(q):
    """Matriz de inercia 2x2. Simetrica y definida positiva."""
    q2 = q[1]
    c2 = np.cos(q2)
    m11 = M1*LC1**2 + I1 + M2*(L1**2 + LC2**2 + 2*L1*LC2*c2) + I2
    m12 = M2*(LC2**2 + L1*LC2*c2) + I2
    m22 = M2*LC2**2 + I2
    return np.array([[m11, m12], [m12, m22]])


def C_matriz(q, qd):
    """
    Matriz de Coriolis y fuerza centrifuga.

    h = -m2 L1 Lc2 sin(q2) es el unico simbolo de Christoffel no nulo
    de un 2R planar.
    """
    q2 = q[1]
    h = -M2*L1*LC2*np.sin(q2)
    return np.array([[h*qd[1], h*(qd[0] + qd[1])],
                     [-h*qd[0], 0.0]])


def g_vector(q):
    """Par gravitatorio. q1 se mide desde la horizontal."""
    q1, q2 = q
    g1 = (M1*LC1 + M2*L1)*G*np.cos(q1) + M2*LC2*G*np.cos(q1 + q2)
    g2 = M2*LC2*G*np.cos(q1 + q2)
    return np.array([g1, g2])


def torque(q, qd, qdd):
    """Devuelve (tau, tau_inercial, tau_coriolis, tau_gravedad)."""
    ti = M_matriz(q) @ qdd
    tc = C_matriz(q, qd) @ qd
    tg = g_vector(q)
    return ti + tc + tg, ti, tc, tg


def torque_trayectoria(traj, t, idx=(1, 2)):
    """
    Evalua tau(t) a lo largo de una trayectoria LSPB completa.

    idx selecciona que juntas del robot alimentan el modelo 2R;
    por defecto las juntas 2 y 3 (indices 1 y 2).
    """
    q, qd, qdd = traj(t)
    n = len(np.atleast_1d(t))
    out = {k: np.zeros((n, 2)) for k in ("tau", "inercial", "coriolis", "gravedad")}
    for j in range(n):
        qq = q[j, list(idx)]
        vv = qd[j, list(idx)]
        aa = qdd[j, list(idx)]
        tau, ti, tc, tg = torque(qq, vv, aa)
        out["tau"][j] = tau
        out["inercial"][j] = ti
        out["coriolis"][j] = tc
        out["gravedad"][j] = tg
    return out


def energia(q, qd):
    """Energia total (cinetica + potencial), para validar el modelo."""
    T = 0.5 * qd @ M_matriz(q) @ qd
    q1, q2 = q
    h1 = LC1*np.sin(q1)
    h2 = L1*np.sin(q1) + LC2*np.sin(q1 + q2)
    return T + G*(M1*h1 + M2*h2)


if __name__ == "__main__":
    print("\n   Validacion del modelo dinamico 2R\n")

    rng = np.random.default_rng(5)
    peor_sim, peor_def = 0.0, np.inf
    for _ in range(500):
        q = rng.uniform(-np.pi, np.pi, 2)
        M = M_matriz(q)
        peor_sim = max(peor_sim, abs(M[0, 1] - M[1, 0]))
        peor_def = min(peor_def, np.linalg.eigvalsh(M).min())
    print(f"  asimetria maxima de M      {peor_sim:.3e}")
    print(f"  autovalor minimo de M      {peor_def:.4f}  (debe ser > 0)")

    # Antisimetria de Mdot - 2C: propiedad estructural de todo manipulador
    q = np.array([0.4, -0.7]); qd = np.array([1.1, -0.6]); h = 1e-6
    Md = (M_matriz(q + h*qd) - M_matriz(q - h*qd)) / (2*h)
    S = Md - 2*C_matriz(q, qd)
    print(f"  ||Mdot - 2C + (.)^T||      {np.abs(S + S.T).max():.3e}  (debe ser ~0)")

    # Conservacion de energia: sistema libre bajo gravedad, tau = 0
    q = np.array([0.3, 0.5]); qd = np.zeros(2)
    E0 = energia(q, qd)
    dt = 1e-5
    for _ in range(200000):
        qdd = np.linalg.solve(M_matriz(q), -C_matriz(q, qd) @ qd - g_vector(q))
        qd = qd + qdd*dt
        q = q + qd*dt
    deriva = abs(energia(q, qd) - E0) / abs(E0)
    print(f"  deriva de energia en 2 s   {deriva:.3e}  (relativa)")
    print("  RESULTADO:", "OK" if peor_def > 0 and deriva < 1e-3 else "REVISAR")
    print()
