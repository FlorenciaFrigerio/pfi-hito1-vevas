#!/usr/bin/env python3
"""
Perfiles de velocidad LSPB (Linear Segment with Parabolic Blends).

Grupo VEVAS
"""

import numpy as np


class LSPB:
    """
    Perfil trapezoidal de velocidad para una junta.

        q(0) = q0,  q(T) = qf,  qd(0) = qd(T) = 0

    Fase 1  (0 <= t < tc)      : aceleracion constante +qdd
    Fase 2  (tc <= t < T - tc) : velocidad constante de crucero
    Fase 3  (T - tc <= t <= T) : desaceleracion constante -qdd

    Condicion de existencia del tramo de crucero:
        |qf - q0| >= V^2 / A       (si no, el perfil degenera en triangular)
    """

    def __init__(self, q0, qf, V, A):
        self.q0, self.qf = float(q0), float(qf)
        self.h = self.qf - self.q0
        self.sgn = np.sign(self.h) if self.h != 0 else 1.0
        V, A = abs(V), abs(A)

        if abs(self.h) < 1e-12:                 # sin movimiento
            self.V = self.A = 0.0
            self.tc = self.T = 0.0
            self.triangular = False
            return

        if abs(self.h) * A >= V * V:            # hay tramo de crucero
            self.V, self.A = V, A
            self.tc = V / A
            self.T = abs(self.h) / V + self.tc
            self.triangular = False
        else:                                   # perfil triangular
            self.A = A
            self.tc = np.sqrt(abs(self.h) / A)
            self.T = 2 * self.tc
            self.V = A * self.tc
            self.triangular = True

    @classmethod
    def desde_tiempo_final(cls, q0, qf, tf, acc):
        """
        Parametrizacion (tf, qdd_c) de la hoja de trabajo W09-S01.

        En vez de fijar velocidad de crucero y aceleracion, se fija el
        TIEMPO TOTAL y la aceleracion. El tiempo de mezcla sale de
        resolver la cuadratica que impone q(tf) = qf:

            tc = tf/2 - (1/2) sqrt( tf^2 - 4(qf - q0)/qdd_c )

        Condicion de factibilidad: el radicando debe ser no negativo,

            |qdd_c| >= 4 |qf - q0| / tf^2

        Si no se cumple, no existe perfil trapezoidal que recorra esa
        distancia en ese tiempo con esa aceleracion: el robot no alcanza
        a llegar. Se lanza ValueError en lugar de devolver NaN.
        """
        h = float(qf) - float(q0)
        acc = abs(float(acc))
        if abs(h) < 1e-12:
            return cls(q0, qf, 1.0, acc)

        acc_min = 4.0 * abs(h) / (tf * tf)
        if acc < acc_min - 1e-12:
            raise ValueError(
                f"Perfil no factible: con tf = {tf} s y |h| = {abs(h):.4f} rad "
                f"se requiere |qdd_c| >= {acc_min:.4f} rad/s^2, "
                f"pero se dio {acc:.4f} rad/s^2.")

        rad = tf*tf - 4.0*abs(h)/acc
        tc = 0.5*tf - 0.5*np.sqrt(max(rad, 0.0))
        V = acc * tc                      # velocidad de crucero resultante

        obj = cls(q0, qf, V, acc)
        obj.acc_min = acc_min
        return obj

    def __call__(self, t):
        """Devuelve (q, qd, qdd) evaluado en t (escalar o arreglo)."""
        t = np.atleast_1d(np.asarray(t, float))
        q = np.full_like(t, self.q0)
        qd = np.zeros_like(t)
        qdd = np.zeros_like(t)
        if self.T == 0:
            return q, qd, qdd

        s, A, V, tc, T = self.sgn, self.A, self.V, self.tc, self.T
        f1 = t < tc
        f2 = (t >= tc) & (t < T - tc)
        f3 = (t >= T - tc) & (t <= T)
        f4 = t > T

        q[f1] = self.q0 + s * 0.5 * A * t[f1]**2
        qd[f1] = s * A * t[f1]
        qdd[f1] = s * A

        q[f2] = self.q0 + s * (V * t[f2] - 0.5 * V * V / A)
        qd[f2] = s * V

        tr = T - t[f3]
        q[f3] = self.qf - s * 0.5 * A * tr**2
        qd[f3] = s * A * tr
        qdd[f3] = -s * A

        q[f4] = self.qf
        return q, qd, qdd


class LSPBSincronizado:
    """
    Seis perfiles LSPB con tiempo final comun.

    Se calcula el T individual de cada junta, se toma el mayor, y se
    reescalan las demas reduciendo su velocidad de crucero para que
    todas arranquen y lleguen a la vez. Asi el movimiento del efector es
    coordinado y no una sucesion de tramos descoordinados.
    """

    def __init__(self, q0, qf, V, A):
        q0, qf = np.asarray(q0, float), np.asarray(qf, float)
        V, A = np.asarray(V, float), np.asarray(A, float)
        perfiles = [LSPB(q0[i], qf[i], V[i], A[i]) for i in range(len(q0))]
        self.T = max(p.T for p in perfiles)

        self.perfiles = []
        for i, p in enumerate(perfiles):
            if p.T == 0 or abs(self.T - p.T) < 1e-12:
                self.perfiles.append(p)
                continue
            # Reescalado: se busca V' tal que el perfil dure exactamente T.
            h, a = abs(p.h), A[i]
            disc = a*a*self.T*self.T - 4*a*h
            V_new = 0.5 * (a*self.T - np.sqrt(max(disc, 0.0)))
            self.perfiles.append(LSPB(q0[i], qf[i], V_new, a))

    def __call__(self, t):
        res = [p(t) for p in self.perfiles]
        q = np.stack([r[0] for r in res], axis=-1)
        qd = np.stack([r[1] for r in res], axis=-1)
        qdd = np.stack([r[2] for r in res], axis=-1)
        return q, qd, qdd


def comprobar_continuidad(perfil, n=20001):
    """Verifica continuidad de q y qd, y que se respeten los limites."""
    t = np.linspace(0, perfil.T, n)
    q, qd, qdd = perfil(t)
    q, qd = np.atleast_2d(q.T).T, np.atleast_2d(qd.T).T
    salto_q = np.abs(np.diff(q, axis=0)).max()
    salto_qd = np.abs(np.diff(qd, axis=0)).max()
    dt = t[1] - t[0]
    return {
        "salto_max_q": float(salto_q),
        "salto_max_qd": float(salto_qd),
        "qd_max": float(np.abs(qd).max()),
        "qdd_max": float(np.abs(qdd).max()),
        "q_inicial_ok": bool(np.allclose(q[0], [p.q0 for p in perfil.perfiles])),
        "q_final_ok": bool(np.allclose(q[-1], [p.qf for p in perfil.perfiles])),
        "qd_extremos_nulos": bool(np.abs(qd[0]).max() < 1e-9
                                  and np.abs(qd[-1]).max() < 1e-9),
        "dt": float(dt),
    }


if __name__ == "__main__":
    q0 = np.zeros(6)
    qf = np.array([0.8, -0.5, 0.6, 0.3, 0.9, -0.4])
    V = np.array([1.0, 1.0, 1.2, 1.5, 1.5, 2.0])
    A = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])

    tr = LSPBSincronizado(q0, qf, V, A)
    print(f"\n   LSPB sincronizado   T = {tr.T:.4f} s\n")
    for i, p in enumerate(tr.perfiles):
        tipo = "triangular" if p.triangular else "trapezoidal"
        print(f"  J{i+1}  h = {p.h:+.3f} rad   tc = {p.tc:.4f} s   "
              f"V = {p.V:.4f} rad/s   {tipo}")

    c = comprobar_continuidad(tr)
    print(f"\n  salto maximo en q    {c['salto_max_q']:.3e} rad")
    print(f"  salto maximo en qd   {c['salto_max_qd']:.3e} rad/s")
    print(f"  extremos con qd = 0  {c['qd_extremos_nulos']}")
    print(f"  q(0) y q(T) exactos  {c['q_inicial_ok']} / {c['q_final_ok']}")
    print()
