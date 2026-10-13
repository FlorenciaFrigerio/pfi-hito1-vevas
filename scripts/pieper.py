#!/usr/bin/env python3
"""
Cinematica inversa analitica del ABB IRB 120 por desacoplamiento de Pieper.

Grupo VEVAS
"""

import numpy as np
from fk_irb120 import DH_TABLE, JOINT_LIMITS, forward_kinematics

d1 = DH_TABLE[0][1]      # 0.290
a2 = DH_TABLE[1][2]      # 0.270
a3 = DH_TABLE[2][2]      # 0.070
d4 = DH_TABLE[3][1]      # 0.302
d6 = DH_TABLE[5][1]      # 0.072

k = np.hypot(a3, d4)             # longitud equivalente antebrazo
psi = np.arctan2(d4, a3)         # angulo de montaje del antebrazo


def wrist_center(T):
    """p_w = p - d6 * R[:,2]  (la junta 6 solo traslada d6 sobre z6)."""
    return T[:3, 3] - d6 * T[:3, 2]


def _R03(q1, q2, q3):
    from fk_irb120 import dh_matrix
    T = np.eye(4)
    for i, qi in enumerate([q1, q2, q3]):
        off, d, a, al = DH_TABLE[i]
        T = T @ dh_matrix(qi + off, d, a, al)
    return T[:3, :3]


def pieper(T_d, tol=1e-9):
    """Devuelve la lista de ramas q (hasta 8) que alcanzan la pose T_d."""
    pw = wrist_center(T_d)
    xw, yw, zw = pw
    soluciones = []

    for signo_hombro in (+1, -1):
        if signo_hombro > 0:
            q1 = np.arctan2(yw, xw)
            r = np.hypot(xw, yw)
        else:
            q1 = np.arctan2(yw, xw) + np.pi
            r = -np.hypot(xw, yw)
        q1 = np.arctan2(np.sin(q1), np.cos(q1))
        s = zw - d1

        D = (r*r + s*s - a2*a2 - k*k) / (2.0*a2*k)
        if abs(D) > 1.0 + 1e-9:
            continue                      # fuera del espacio de trabajo
        D = np.clip(D, -1.0, 1.0)

        for signo_codo in (+1, -1):
            gamma = signo_codo * np.arccos(D)     # gamma = q3 + psi
            q3 = gamma - psi
            U = a2 + k*np.cos(gamma)
            V = k*np.sin(gamma)
            q2 = np.arctan2(U*r - V*s, V*r + U*s)

            R36 = _R03(q1, q2, q3).T @ T_d[:3, :3]
            c5 = np.clip(R36[2, 2], -1.0, 1.0)

            if abs(abs(c5) - 1.0) < tol:          # SINGULARIDAD q5 = 0
                q5 = 0.0 if c5 > 0 else np.pi
                q4 = 0.0
                q6 = np.arctan2(R36[1, 0], R36[0, 0]) * (1 if c5 > 0 else -1)
                soluciones.append(np.array([q1, q2, q3, q4, q5, q6]))
                continue

            for signo_muneca in (+1, -1):
                q5 = signo_muneca * np.arccos(c5)
                s5 = np.sin(q5)
                q4 = np.arctan2(-R36[1, 2]/s5, -R36[0, 2]/s5)
                q6 = np.arctan2(-R36[2, 1]/s5,  R36[2, 0]/s5)
                soluciones.append(np.array([q1, q2, q3, q4, q5, q6]))

    return soluciones


def dentro_de_limites(q):
    return bool(np.all(q >= JOINT_LIMITS[:, 0]) and np.all(q <= JOINT_LIMITS[:, 1]))


def etiquetar(q):
    """
    Nombre geometrico de la rama, para poder discutirla en la defensa.

    codo: el signo de q3 separa las dos posturas del triangulo
          hombro-codo-muneca.
    muneca: el signo de q5 distingue las dos formas de alcanzar la misma
          orientacion, relacionadas por q4 -> q4+pi, q5 -> -q5, q6 -> q6+pi.
    hombro: si el brazo apunta hacia el objetivo o lo alcanza girando la
          base media vuelta.
    """
    codo = "arriba" if q[2] >= 0 else "abajo"
    muneca = "no invertida" if q[4] >= 0 else "invertida"

    # El hombro no se lee de q2: se compara q1 contra el acimut real del
    # centro de muneca. Si coinciden, la base apunta hacia el objetivo;
    # si difieren pi, lo alcanza girando media vuelta y estirandose hacia
    # atras.
    pw = wrist_center(forward_kinematics(q))
    azim = np.arctan2(pw[1], pw[0])
    dif = abs(np.arctan2(np.sin(q[0] - azim), np.cos(q[0] - azim)))
    hombro = "frontal" if dif < np.pi/2 else "posterior"

    return f"codo {codo} / muneca {muneca} / hombro {hombro}"


def filtrar(soluciones, T_d, tol_pose=1e-6):
    """
    Descarta las ramas inservibles. Tres filtros, en este orden:

      1. Numerico   -- elimina NaN, que aparecen cuando una rama cae
                       fuera del espacio de trabajo alcanzable.
      2. Cinematico -- verifica T_0^6(q) ~ T_d. Una rama que no
                       reproduce la pose es un error algebraico, no una
                       solucion alternativa.
      3. Fisico     -- descarta las que violan los limites articulares
                       del datasheet de ABB. Son matematicamente validas
                       pero mecanicamente imposibles.

    Devuelve (admisibles, rechazadas) para poder documentar el descarte
    en vez de ocultarlo.
    """
    admisibles, rechazadas = [], []
    for q in soluciones:
        if not np.all(np.isfinite(q)):
            rechazadas.append((q, "no finita"))
            continue
        ep, eR = error_pose(q, T_d)
        if max(ep, eR) > tol_pose:
            rechazadas.append((q, f"FK no coincide (dE = {ep:.2e} m)"))
            continue
        if not dentro_de_limites(q):
            fuera = [f"J{i+1}" for i in range(6)
                     if not (JOINT_LIMITS[i, 0] <= q[i] <= JOINT_LIMITS[i, 1])]
            rechazadas.append((q, "fuera de limites: " + ", ".join(fuera)))
            continue
        admisibles.append(q)
    return admisibles, rechazadas


def elegir(admisibles, q_actual, pesos=None):
    """
    Selecciona una rama entre las admisibles por minimo desplazamiento
    articular ponderado respecto de la configuracion actual:

        k* = argmin_k  sum_i  w_i (q_i^(k) - q_i^actual)^2

    Los pesos penalizan mas el movimiento de las juntas grandes: mover
    la base arrastra todo el brazo, mientras que girar la brida casi no
    cuesta. Sin pesos, el criterio elegiria ramas que hacen girar media
    vuelta la base para ahorrar unos grados en la muneca.

    Este criterio tambien evita saltos de configuracion entre dos puntos
    consecutivos de una trayectoria, que en el robot real se traducen en
    movimientos bruscos e impredecibles.
    """
    if not admisibles:
        return None, None
    if pesos is None:
        pesos = np.array([4.0, 3.0, 2.0, 1.0, 1.0, 0.5])
    q_actual = np.asarray(q_actual, float)
    costos = [float(np.sum(pesos * (q - q_actual)**2)) for q in admisibles]
    k = int(np.argmin(costos))
    return admisibles[k], costos[k]


def resolver(T_d, q_actual=None, verbose=False):
    """Pipeline completo: Pieper -> filtrado -> seleccion."""
    if q_actual is None:
        q_actual = np.zeros(6)
    crudas = pieper(T_d)
    admisibles, rechazadas = filtrar(crudas, T_d)
    q_sel, costo = elegir(admisibles, q_actual)

    if verbose:
        print(f"  ramas calculadas  {len(crudas)}")
        print(f"  admisibles        {len(admisibles)}")
        print(f"  rechazadas        {len(rechazadas)}")
        for q, motivo in rechazadas:
            print(f"     - {motivo}")
        if q_sel is not None:
            print(f"  elegida           {etiquetar(q_sel)}  (costo {costo:.4f})")
    return q_sel, admisibles, rechazadas


def error_pose(q, T_d):
    T = forward_kinematics(q)
    ep = np.linalg.norm(T[:3, 3] - T_d[:3, 3])
    eR = np.linalg.norm(T[:3, :3].T @ T_d[:3, :3] - np.eye(3), "fro")
    return ep, eR


if __name__ == "__main__":
    rng = np.random.default_rng(3)
    peor_p = peor_R = 0.0
    sin_sol = 0
    conteo = []
    for _ in range(2000):
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        Td = forward_kinematics(q)
        sols = pieper(Td)
        validas = [s for s in sols if max(error_pose(s, Td)) < 1e-6]
        conteo.append(len(validas))
        if not validas:
            sin_sol += 1
            continue
        for s in validas:
            ep, eR = error_pose(s, Td)
            peor_p, peor_R = max(peor_p, ep), max(peor_R, eR)

    print(f"\n   Pieper: verificacion FK sobre 2000 poses\n")
    print(f"  ramas validas por pose (media)  {np.mean(conteo):.2f}")
    print(f"  poses sin solucion              {sin_sol}")
    print(f"  error posicional maximo         {peor_p:.3e} m")
    print(f"  error de orientacion maximo     {peor_R:.3e}")
    print("  RESULTADO:", "OK" if peor_p < 1e-9 and sin_sol == 0 else "REVISAR")
    print()
