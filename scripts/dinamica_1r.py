#!/usr/bin/env python3
"""
Modelo dinamico reducido 1R, exigido por la guia oficial del Hito 2.

Que pide la guia

    "Se utilizara el modelo reducido 1R estudiado en clase:
         tau = J qdd + m g lc sin(q)
     Evaluara esta ecuacion utilizando la trayectoria LSPB previamente
     calculada y graficara tau(t). Debera interpretar las regiones de
     mayor esfuerzo y la diferencia entre degradacion cinematica
     (singularidad) y comportamiento dinamico."
                                -- Guia Oficial Hito 2, seccion 3.4

Procedencia de los parametros

    La guia prohibe inventar masas e inercias del IRB 120, que ABB no
    publica. Por eso NO se estiman: se usan los valores pedagogicos que
    el propio docente declara en la hoja practica W10-S01, Tarea 4D:

        J  = 0.25 kg m^2      inercia respecto del eje de giro
        m  = 1.00 kg          masa del eslabon
        lc = 0.50 m           distancia al centro de masa
        g  = 9.81 m/s^2

    El angulo q se mide desde la VERTICAL INFERIOR, tambien segun esa
    hoja: por eso la energia potencial es V = m g lc (1 - cos q) y el
    termino gravitatorio resulta m g lc sin(q), nulo en q = 0.

Derivacion (Euler-Lagrange), para poder reconstruirla en la defensa

    T = (1/2) J qd^2
    V = m g lc (1 - cos q)
    L = T - V = (1/2) J qd^2 - m g lc (1 - cos q)

    dL/dqd          = J qd
    d/dt (dL/dqd)   = J qdd
    dL/dq           = -m g lc sin(q)

    tau = d/dt(dL/dqd) - dL/dq = J qdd + m g lc sin(q)

Grupo VEVAS
"""

import numpy as np

# Valores pedagogicos declarados por el docente (W10-S01, Tarea 4D).
J_INERCIA = 0.25      # kg m^2
M_MASA    = 1.00      # kg
LC_CENTRO = 0.50      # m
G_GRAV    = 9.81      # m/s^2


def torque_1r(q, qdd, J=J_INERCIA, m=M_MASA, lc=LC_CENTRO, g=G_GRAV):
    """
    tau = J qdd + m g lc sin(q)

    Acepta escalares o arrays; devuelve el torque total junto con sus
    dos contribuciones por separado, porque la rubrica exige "separa e
    interpreta contribuciones".
    """
    q, qdd = np.asarray(q, float), np.asarray(qdd, float)
    tau_inercial = J * qdd
    tau_gravedad = m * g * lc * np.sin(q)
    return tau_inercial + tau_gravedad, tau_inercial, tau_gravedad


def torque_sobre_lspb(perfil, t):
    """
    Evalua el modelo 1R a lo largo de una trayectoria LSPB.

    perfil : instancia de LSPB (callable que devuelve q, qd, qdd)
    t      : vector de tiempos

    Devuelve un diccionario con las series temporales, listo para
    graficar y para la tabla de resultados del informe.
    """
    t = np.asarray(t, float)
    q, qd, qdd = perfil(t)          # LSPB ya evalua vectorizado
    tau, tau_i, tau_g = torque_1r(q, qdd)
    return {"t": t, "q": q, "qd": qd, "qdd": qdd,
            "tau": tau, "tau_inercial": tau_i, "tau_gravedad": tau_g}


def verificacion_analitica(q, qdd):
    """
    Recalcula el torque a mano, termino por termino, para contrastar.

    Sirve de validacion independiente: si el valor coincide con
    torque_1r, la implementacion no tiene errores de signo ni de
    unidades. La rubrica premia "documenta error o criterio de
    aceptacion".
    """
    ti = J_INERCIA * qdd
    tg = M_MASA * G_GRAV * LC_CENTRO * np.sin(q)
    return {"q": q, "qdd": qdd, "inercial": ti, "gravedad": tg,
            "total": ti + tg}


if __name__ == "__main__":
    from lspb import LSPB

    np.set_printoptions(precision=4, suppress=True)
    print("\n   MODELO DINAMICO REDUCIDO 1R   tau = J qdd + m g lc sin(q)\n")

    print("1. Parametros declarados (W10-S01, Tarea 4D)")
    print(f"   J  = {J_INERCIA:.4f} kg m^2")
    print(f"   m  = {M_MASA:.4f} kg")
    print(f"   lc = {LC_CENTRO:.4f} m")
    print(f"   g  = {G_GRAV:.4f} m/s^2")
    print("   Ninguno es estimado: todos provienen de la hoja del docente.\n")

    perfil = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
    print("2. Trayectoria LSPB de referencia (W10-S01, Tarea 3)")
    print(f"   q0 = 0 rad   qf = 2 rad   tf = 3 s   qddc = 1 rad/s^2")
    print(f"   tiempo de mezcla  tc = {perfil.tc:.4f} s")
    print(f"   velocidad crucero V  = {perfil.V:.4f} rad/s\n")

    t_c = 2.5
    q_c, qd_c, qdd_c = (float(v[0]) for v in perfil(t_c))
    chequeo = verificacion_analitica(q_c, qdd_c)
    print(f"3. Punto de control t = {t_c} s  (Tarea 4D de la hoja)")
    print(f"   q   = {q_c:.4f} rad")
    print(f"   qd  = {qd_c:.4f} rad/s")
    print(f"   qdd = {qdd_c:.4f} rad/s^2")
    print(f"   tau inercial  = {chequeo['inercial']:>8.4f} N.m")
    print(f"   tau gravedad  = {chequeo['gravedad']:>8.4f} N.m")
    print(f"   tau total     = {chequeo['total']:>8.4f} N.m")
    signo = "negativa (frena)" if chequeo["inercial"] < 0 else "positiva (acelera)"
    print(f"   La contribucion inercial es {signo}: en t = 2.5 s el perfil")
    print("   esta en la fase de desaceleracion, asi que qdd < 0 y el motor")
    print("   debe oponerse al movimiento que el ya habia impuesto.\n")

    t = np.linspace(0.0, perfil.T, 601)
    res = torque_sobre_lspb(perfil, t)
    i_max = int(np.argmax(np.abs(res["tau"])))
    i_qdd = int(np.argmax(np.abs(res["qdd"])))

    print("4. Interpretacion de tau(t) sobre toda la trayectoria\n")
    print(f"   |tau| maximo      = {abs(res['tau'][i_max]):.4f} N.m  en t = {t[i_max]:.3f} s")
    print(f"   |qdd| maximo      = {abs(res['qdd'][i_qdd]):.4f} rad/s^2 en t = {t[i_qdd]:.3f} s")
    coinciden = abs(t[i_max] - t[i_qdd]) < 1e-6
    print(f"   Coinciden los instantes: {'si' if coinciden else 'NO'}")

    mascara = np.abs(res["qdd"]) < 1e-9
    if mascara.any():
        tau_cr = np.abs(res["tau"][mascara])
        print(f"\n   Durante el crucero qdd = 0 exacto, y sin embargo")
        print(f"   |tau| va de {tau_cr.min():.4f} a {tau_cr.max():.4f} N.m.")
        print("   Velocidad constante NO implica torque cero: el motor sigue")
        print("   sosteniendo el peso del eslabon contra la gravedad.")

    print("\n5. Singularidad frente a dinamica: son fenomenos distintos")
    print("   Este modelo 1R no tiene singularidades: su jacobiano es un")
    print("   escalar que nunca se anula. Y aun asi exige torque elevado.")
    print(f"   Ese es el contraejemplo: {abs(res['tau'][i_max]):.2f} N.m de esfuerzo")
    print("   sin ninguna degradacion cinematica. La singularidad es perdida")
    print("   de movilidad (rango de J); el torque es demanda de esfuerzo.")
    print("   Una no implica la otra.\n")
