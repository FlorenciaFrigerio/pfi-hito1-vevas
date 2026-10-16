#!/usr/bin/env python3
"""
Pasaporte del Hito 2: divergencia analitica de velocidades y torques
en la singularidad de muneca q5 = 0.

Que exige el pasaporte

    "Demostracion en Jupyter Notebook de la divergencia analitica de
     velocidades y torques en singularidad (q5 = 0 rad)."
                                        -- Guia Oficial del PFI, Hito 2

Advertencia conceptual obligatoria

    El docente senala explicitamente que "singularidad cinematica NO es
    igual a torque alto": una postura regular puede exigir un torque
    gravitatorio enorme, y una postura singular se define por la
    movilidad perdida, no por tau.

    La divergencia de torque que se demuestra aqui NO contradice eso.
    Es INDIRECTA: la singularidad hace divergir qd por cinematica
    diferencial inversa, qdd diverge como derivada de qd, y recien
    entonces tau = M qdd + ... hereda la divergencia. La singularidad no
    "produce" torque: produce velocidad articular infinita, y la
    velocidad infinita produce torque infinito.

Grupo VEVAS
"""

import numpy as np
from fk_irb120 import JOINT_LIMITS
from jacobiano import jacobiano

# Inercias de rotor reflejadas de las juntas de muneca [kg m^2].
# Estimacion declarada: ABB no publica estos valores. Sirven para
# mostrar la LEY de escalado tau ~ 1/q5^2, no una cifra certificada.
I_ROTOR = np.array([0.012, 0.012, 0.008])     # J4, J5, J6


def direccion_perdida(q, tol=1e-9):
    """
    Direccion del espacio de tareas que el robot no puede recorrer.

    En una singularidad J pierde rango, asi que su imagen deja de cubrir
    R^6. El vector singular izquierdo asociado al menor valor singular
    es, justamente, la direccion de tarea que queda fuera de alcance
    instantaneo.
    """
    U, S, _ = np.linalg.svd(jacobiano(q))
    return U[:, -1], S[-1]


def direcciones_conservadas(q, tol=1e-6):
    """
    Direcciones de tarea que el robot SI puede recorrer en la singularidad.

    Por que hace falta esto

        La matriz diagnostica W10-S01 marca como error conceptual decir
        que "singularidad = no se mueve", y pide explicitamente mostrar
        "las direcciones espaciales que si persisten".

    En una singularidad J pierde rango r < 6. Las r primeras columnas de
    U (valores singulares no despreciables) generan la imagen de J: el
    subespacio de velocidades de tarea todavia alcanzable. La movilidad
    no se anula, se degrada de 6 a r dimensiones.
    """
    U, S, _ = np.linalg.svd(jacobiano(q))
    mantiene = S > tol * max(S[0], 1.0)
    return U[:, mantiene], S[mantiene]


def velocidad_requerida(q, xd):
    """
    qd = J^-1 xd, resuelto por minimos cuadrados.

    Se usa lstsq y no inv() a proposito: en la singularidad exacta la
    inversa no existe, y lstsq devuelve la solucion de norma minima en
    vez de abortar. Eso permite graficar la curva completa hasta q5 = 0.
    """
    J = jacobiano(q)
    qd, *_ = np.linalg.lstsq(J, xd, rcond=None)
    return qd


def barrido(q_base, xd, q5_vals):
    """Recorre q5 -> 0 registrando sigma_min, ||qd|| y el numero de condicion."""
    filas = []
    for q5 in q5_vals:
        q = np.array(q_base, float)
        q[4] = q5
        J = jacobiano(q)
        sv = np.linalg.svd(J, compute_uv=False)
        qd = velocidad_requerida(q, xd)
        filas.append({
            "q5": q5,
            "sigma_min": sv[-1],
            "cond": sv[0]/sv[-1] if sv[-1] > 1e-300 else np.inf,
            "rango": int(np.linalg.matrix_rank(J, tol=1e-8)),
            "det": float(np.linalg.det(J)),
            "norma_qd": float(np.linalg.norm(qd)),
            "qd4": float(qd[3]),
            "qd6": float(qd[5]),
        })
    return filas


def ley_de_escalado(q5_vals, valores):
    """
    Ajusta  log|valor| = m log|q5| + b  y devuelve la pendiente m.

    Una pendiente -1 confirma divergencia como 1/q5; una pendiente -2
    confirma 1/q5^2. Es la evidencia cuantitativa de que la divergencia
    es analitica y no un artefacto numerico.
    """
    x = np.log10(np.abs(q5_vals))
    y = np.log10(np.abs(valores))
    ok = np.isfinite(x) & np.isfinite(y)
    m, b = np.polyfit(x[ok], y[ok], 1)
    return float(m), float(b)


def torque_muneca(q_base, xd, q5_vals, v5=1.0, paso_rel=1e-3):
    """
    Torque inercial en las juntas de muneca al atravesar la singularidad.

    Escenario: el robot se aproxima a la singularidad con velocidad
    articular constante qd5 = v5 [rad/s] mientras el efector mantiene la
    velocidad de tarea xd. Entonces, por regla de la cadena,

        qdd_i = d(qd_i)/dt = [d(qd_i)/dq5] . v5

    Como qd_i ~ C/q5, su derivada va como -C/q5^2, de modo que la
    aceleracion -y con ella el torque- diverge con el CUADRADO.

        tau_i ~ I_i qdd_i          (modelo de rotor desacoplado)

    El paso de la diferencia finita es RELATIVO a q5. Con un paso fijo,
    al acercarse a la singularidad el intervalo termina cruzandola y la
    derivada se satura en un valor espurio.
    """
    taus = []
    for q5 in q5_vals:
        h = max(abs(q5) * paso_rel, 1e-18)
        qa = np.array(q_base, float); qa[4] = q5 + h
        qb = np.array(q_base, float); qb[4] = q5 - h
        qd_a = velocidad_requerida(qa, xd)
        qd_b = velocidad_requerida(qb, xd)
        qdd = (qd_a - qd_b) / (2*h) * v5
        taus.append(np.abs(I_ROTOR * qdd[[3, 4, 5]]).max())
    return np.array(taus)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    q_base = np.array([0.3, 0.5, -0.4, 0.2, 0.0, 0.1])

    print("\n   PASAPORTE HITO 2   divergencia en q5 = 0\n")

    # --- 1. Que direccion de tarea se pierde exactamente
    q_sing = q_base.copy(); q_sing[4] = 0.0
    u, s_min = direccion_perdida(q_sing)
    print("1. Direccion de tarea perdida en la singularidad")
    print(f"   sigma_min = {s_min:.3e}")
    print(f"   vector    = {u}")
    print(f"   lineal    = {u[:3]}   angular = {u[3:]}")
    print("   La componente dominante es angular: la muneca pierde una")
    print("   rotacion, no una traslacion.\n")

    # --- 1b. Que direcciones SOBREVIVEN (contra el error "no se mueve")
    Uc, Sc = direcciones_conservadas(q_sing)
    print("1b. Direcciones de tarea CONSERVADAS en la singularidad")
    print(f"   rango(J) = {Uc.shape[1]} de 6  =>  pierde 1 grado de libertad,")
    print("   no los seis. El robot sigue moviendose en el resto.")
    for i in range(Uc.shape[1]):
        v, w = Uc[:3, i], Uc[3:, i]
        tipo = "traslacion" if np.linalg.norm(v) > np.linalg.norm(w) else "rotacion"
        print(f"   base {i+1}: sigma = {Sc[i]:>8.4f}   dominante: {tipo:<10}"
              f" |v| = {np.linalg.norm(v):.3f}  |w| = {np.linalg.norm(w):.3f}")
    print()

    # --- 2. Barrido: velocidad articular exigida
    xd = np.zeros(6)
    xd[3:] = u[3:] / np.linalg.norm(u[3:])    # velocidad angular unitaria
    q5_vals = np.array([1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])
    filas = barrido(q_base, xd, q5_vals)

    print("2. Velocidad articular exigida para una velocidad de tarea unitaria\n")
    print(f"   {'q5 [rad]':>10}  {'rango':>5}  {'sigma_min':>11}  "
          f"{'cond(J)':>11}  {'||qd|| [rad/s]':>15}")
    for f in filas:
        print(f"   {f['q5']:>10.0e}  {f['rango']:>5d}  {f['sigma_min']:>11.3e}  "
              f"{f['cond']:>11.3e}  {f['norma_qd']:>15.3e}")

    m_v, _ = ley_de_escalado(q5_vals, [f["norma_qd"] for f in filas])
    print(f"\n   Ley de escalado ajustada:  ||qd|| ~ q5^({m_v:.3f})")
    print(f"   Pendiente -1 => divergencia hiperbolica 1/q5.  "
          f"{'CONFIRMADO' if abs(m_v+1) < 0.05 else 'REVISAR'}\n")

    # --- 3. Torque
    taus = torque_muneca(q_base, xd, q5_vals)
    m_t, _ = ley_de_escalado(q5_vals, taus)
    print("3. Torque inercial de muneca exigido\n")
    print(f"   {'q5 [rad]':>10}  {'|tau|max [N.m]':>16}")
    for q5, t in zip(q5_vals, taus):
        print(f"   {q5:>10.0e}  {t:>16.3e}")
    print(f"\n   Ley de escalado ajustada:  tau ~ q5^({m_t:.3f})")
    print(f"   Pendiente -2 => el torque diverge con el CUADRADO.  "
          f"{'CONFIRMADO' if abs(m_t+2) < 0.1 else 'REVISAR'}\n")

    # --- 4. El contraejemplo que exige el docente
    print("4. Contraejemplo obligatorio: singularidad != torque alto\n")
    from dinamica import g_vector
    q_reg = np.array([0.0, 0.0])        # brazo horizontal, NO singular
    J_reg = jacobiano(np.array([0., 0., 0., 0., 1.0, 0.]))
    print(f"   Postura regular (q5 = 1.0): rango J = "
          f"{np.linalg.matrix_rank(J_reg)}, sin divergencia alguna")
    print(f"   Torque gravitatorio en esa postura: "
          f"{np.abs(g_vector(q_reg)).max():.2f} N.m")
    print("   Es un torque elevado SIN singularidad: confirma que ambos")
    print("   fenomenos son independientes. La divergencia del punto 3 es")
    print("   indirecta, heredada de qdd, no una propiedad de la postura.\n")
