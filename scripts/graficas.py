#!/usr/bin/env python3
"""
Figuras del Hito 2.

Paleta categorica validada para daltonismo (deuteranopia, protanopia y
tritanopia) sobre fondo claro: todas las parejas adyacentes superan el
umbral de separacion perceptual. Ademas cada serie lleva etiqueta
directa o leyenda, de modo que la identidad nunca depende solo del color.

Grupo VEVAS
"""

import numpy as np
import matplotlib

# El backend Agg escribe a archivo y no dibuja en pantalla. Se fuerza
# SOLO cuando este modulo se ejecuta como script. Al importarlo desde un
# notebook hay que respetar el backend inline que Jupyter ya configuro,
# porque si se lo pisa las figuras no quedan embebidas en el .ipynb y el
# docente abriria el cuaderno sin ver ninguna grafica.
if __name__ == "__main__":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt

# Paleta categorica (orden fijo, nunca ciclado)
C = ["#3b6ea5", "#d2601a", "#1a9c8f", "#c0392b", "#9b59b6", "#8a8f00"]
TINTA = "#1f2a33"       # texto principal
TINTA_2 = "#5a6672"     # texto secundario
REJILLA = "#dfe3e6"

plt.rcParams.update({
    "font.size": 9,
    "font.family": "serif",
    "axes.edgecolor": TINTA_2,
    "axes.labelcolor": TINTA,
    "text.color": TINTA,
    "xtick.color": TINTA_2,
    "ytick.color": TINTA_2,
    "axes.grid": True,
    "grid.color": REJILLA,
    "grid.linewidth": 0.6,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
})


def _limpiar(ax):
    """Marco recesivo: solo los ejes que se leen."""
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_axisbelow(True)


def _fases(ax, tc, T):
    """Sombrea las tres fases del LSPB y las nombra una sola vez."""
    ax.axvspan(0, tc, color="#000000", alpha=0.04, lw=0)
    ax.axvspan(T - tc, T, color="#000000", alpha=0.04, lw=0)
    ax.axvline(tc, color=TINTA_2, lw=0.8, ls=":")
    ax.axvline(T - tc, color=TINTA_2, lw=0.8, ls=":")


def perfil_lspb(traj, archivo="fig_lspb.pdf", titulo=None):
    """
    Tres paneles apilados: q(t), qd(t), qdd(t), con eje de tiempo comun.

    Se apilan en vez de superponerse porque tienen unidades distintas
    (rad, rad/s, rad/s^2): un solo eje con dos escalas seria enganoso.
    """
    t = np.linspace(0, traj.T, 1200)
    q, qd, qdd = traj(t)

    # Cada junta tiene su propio tiempo de mezcla. Se sombrean las fases
    # de la junta que FIJA el tiempo total (la de mayor tc), y se dice
    # explicitamente: sombrear un tc comun que no existe seria enganoso.
    j_ref = int(np.argmax([p.tc for p in traj.perfiles]))
    tc = traj.perfiles[j_ref].tc

    fig, axs = plt.subplots(3, 1, figsize=(7.0, 6.2), sharex=True)
    etiquetas = [f"J{i+1}" for i in range(q.shape[1])]
    datos = [(q, "$q(t)$  [rad]"), (qd, r"$\dot{q}(t)$  [rad/s]"),
             (qdd, r"$\ddot{q}(t)$  [rad/s$^2$]")]

    for ax, (Y, ylab) in zip(axs, datos):
        _fases(ax, tc, traj.T)
        for j in range(Y.shape[1]):
            ax.plot(t, Y[:, j], color=C[j], lw=2.0, label=etiquetas[j])
        ax.set_ylabel(ylab)
        _limpiar(ax)

    for x, txt in [(tc/2, "acel."), (traj.T/2, "crucero"),
                   (traj.T - tc/2, "desacel.")]:
        axs[0].annotate(txt, xy=(x/traj.T, 1.005), xycoords="axes fraction",
                        ha="center", va="bottom", fontsize=7.5, color=TINTA_2)

    axs[2].set_xlabel(f"$t$  [s]      $T = {traj.T:.3f}$ s")
    fig.text(0.5, -0.012,
             f"Zonas sombreadas: fases de J{j_ref+1}, la junta que fija $T$. "
             f"Cada junta tiene su propio tiempo de mezcla $t_c$.",
             ha="center", fontsize=7.5, color=TINTA_2, style="italic")
    axs[0].legend(ncol=6, frameon=False, fontsize=8,
                  loc="lower center", bbox_to_anchor=(0.5, 1.09))
    if titulo:
        fig.suptitle(titulo, y=0.99, fontsize=10)
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo


def descomposicion_torque(t, res, junta=0, archivo="fig_torque.pdf"):
    """
    Torque total y sus tres contribuciones, para una junta.

    Es la grafica que responde la pregunta del docente: el pico de torque
    NO cae necesariamente donde la aceleracion es maxima, porque la
    gravedad puede dominar en el tramo de crucero.
    """
    tau = res["tau"][:, junta]
    ine = res["inercial"][:, junta]
    cor = res["coriolis"][:, junta]
    gra = res["gravedad"][:, junta]

    # Dos paneles en vez de dos escalas sobre un mismo eje: el termino
    # gravitatorio es un orden de magnitud mayor que los demas, y
    # superponerlos haria invisibles al inercial y al de Coriolis.
    # Un eje doble seria enganoso; dos paneles con el mismo eje de
    # tiempo no lo son.
    fig, axs = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                            gridspec_kw={"height_ratios": [1.5, 1.0]})

    axs[0].plot(t, tau, color=C[3], lw=2.4, label="total")
    axs[0].plot(t, gra, color=C[1], lw=1.6, ls="--", label="gravedad")
    k = int(np.argmax(np.abs(tau)))
    axs[0].plot(t[k], tau[k], "o", color=C[3], ms=7, mec="white", mew=1.5,
                zorder=5)
    axs[0].annotate(f"pico {tau[k]:.2f} N·m en $t$ = {t[k]:.2f} s",
                    xy=(t[k], tau[k]), xytext=(14, -22),
                    textcoords="offset points", fontsize=8, color=TINTA_2,
                    arrowprops=dict(arrowstyle="->", color=TINTA_2, lw=0.8))
    axs[0].set_ylabel(r"$\tau$  [N$\cdot$m]")
    axs[0].legend(frameon=False, fontsize=8, ncol=2,
                  loc="lower center", bbox_to_anchor=(0.5, 1.0))

    axs[1].plot(t, ine, color=C[0], lw=1.8, label="inercial  $M\\ddot{q}$")
    axs[1].plot(t, cor, color=C[2], lw=1.8,
                label="Coriolis  $C\\dot{q}$")
    axs[1].axhline(0, color=TINTA_2, lw=0.8)
    axs[1].set_ylabel(r"$\tau$  [N$\cdot$m]")
    axs[1].set_xlabel("$t$  [s]")
    axs[1].legend(frameon=False, fontsize=8, ncol=2,
                  loc="lower center", bbox_to_anchor=(0.5, 1.0))

    for ax in axs:
        _limpiar(ax)

    fig.text(0.5, -0.02,
             f"Escalas distintas: la gravedad domina con "
             f"{np.abs(gra).max():.1f} N·m frente a "
             f"{np.abs(ine).max():.2f} N·m del término inercial.",
             ha="center", fontsize=7.5, color=TINTA_2, style="italic")
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo


def divergencia(q5_vals, norma_qd, taus, m_v, m_t,
                archivo="fig_divergencia.pdf"):
    """
    El pasaporte, en escala log-log.

    En log-log una ley de potencia es una recta cuya pendiente es el
    exponente. Que los puntos caigan sobre una recta de pendiente -1 y
    -2 es la prueba grafica de que la divergencia es analitica y no un
    accidente numerico.
    """
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.2))

    for ax, y, m, lab, col, uni in [
            (axs[0], norma_qd, m_v, r"$\|\dot{q}\|$", C[0], "rad/s"),
            (axs[1], taus,     m_t, r"$|\tau|_{\max}$", C[3], "N·m")]:
        ax.loglog(q5_vals, y, "o-", color=col, lw=2.0, ms=6,
                  mec="white", mew=1.2)
        ref = y[0] * (q5_vals/q5_vals[0])**round(m)
        ax.loglog(q5_vals, ref, "--", color=TINTA_2, lw=1.0,
                  label=f"pendiente {round(m):+d}")
        ax.set_xlabel("$q_5$  [rad]")
        ax.set_ylabel(f"{lab}  [{uni}]")
        ax.legend(frameon=False, fontsize=8, loc="upper right")
        ax.grid(which="both", color=REJILLA, lw=0.5)
        _limpiar(ax)

    axs[0].set_title("Velocidad articular exigida", fontsize=9, pad=8)
    axs[1].set_title("Torque inercial de muñeca", fontsize=9, pad=8)
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo


def manipulabilidad(t, w, rango, archivo="fig_manipulabilidad.pdf"):
    """Medida de manipulabilidad a lo largo de la trayectoria."""
    fig, ax = plt.subplots(figsize=(7.0, 2.8))
    ax.plot(t, w, color=C[0], lw=2.2)
    ax.fill_between(t, 0, w, color=C[0], alpha=0.10, lw=0)
    k = int(np.argmin(w))
    ax.plot(t[k], w[k], "o", color=C[3], ms=7, mec="white", mew=1.5)
    ax.annotate(f"mínimo {w[k]:.4f}\nrango = {rango[k]}",
                xy=(t[k], w[k]), xytext=(12, 16),
                textcoords="offset points", fontsize=8, color=TINTA_2,
                arrowprops=dict(arrowstyle="->", color=TINTA_2, lw=0.8))
    ax.set_xlabel("$t$  [s]")
    ax.set_ylabel(r"$w = \sqrt{\det(JJ^{T})}$")
    _limpiar(ax)
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo


if __name__ == "__main__":
    from lspb import LSPBSincronizado
    from dinamica import torque_trayectoria
    from jacobiano import jacobiano, medida_manipulabilidad
    import singularidad as sg

    q0 = np.zeros(6)
    qf = np.array([0.8, -0.5, 0.6, 0.3, 0.9, -0.4])
    V = np.array([1.0, 1.0, 1.2, 1.5, 1.5, 2.0])
    A = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])
    traj = LSPBSincronizado(q0, qf, V, A)

    t = np.linspace(0, traj.T, 400)
    res = torque_trayectoria(traj, t)

    f1 = perfil_lspb(traj)
    f2 = descomposicion_torque(t, res, junta=0)

    q_base = np.array([0.3, 0.5, -0.4, 0.2, 0.0, 0.1])
    u, _ = sg.direccion_perdida(np.r_[q_base[:4], 0.0, q_base[5]])
    xd = np.zeros(6); xd[3:] = u[3:]/np.linalg.norm(u[3:])
    q5v = np.array([1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6])
    filas = sg.barrido(q_base, xd, q5v)
    nq = np.array([f["norma_qd"] for f in filas])
    tq = sg.torque_muneca(q_base, xd, q5v)
    mv, _ = sg.ley_de_escalado(q5v, nq)
    mt, _ = sg.ley_de_escalado(q5v, tq)
    f3 = divergencia(q5v, nq, tq, mv, mt)

    q_t, _, _ = traj(t)
    w = np.array([medida_manipulabilidad(q_t[i]) for i in range(len(t))])
    rg = np.array([np.linalg.matrix_rank(jacobiano(q_t[i]), tol=1e-8)
                   for i in range(len(t))])
    f4 = manipulabilidad(t, w, rg)

    print("\n   Figuras generadas\n")
    for f in (f1, f2, f3, f4):
        print(f"   {f}")
    print()
