#!/usr/bin/env python3
"""
Validacion del gemelo digital: arbol TF de ROS contra cinematica analitica.

Este es el PASAPORTE del Hito 1. A diferencia de verify_chain(), que
comprueba la coherencia interna del generador sin salir de Python, este
script interroga al ROS que esta corriendo: publica una configuracion en
/joint_states, espera a que robot_state_publisher recomponga el arbol,
lee la transformada base_link -> tool0 por tf2 y la contrasta con
forward_kinematics().

Sincronizacion

    La trampa de este script es el desfase temporal. Si se publica q y se
    lee TF tras una espera fija, tf2 puede devolver todavia la
    transformada de la configuracion ANTERIOR, porque
    robot_state_publisher aun no proceso el mensaje. El error resultante
    es del orden del alcance del robot, no del epsilon de maquina.

    La solucion es no confiar en una espera fija: se sella cada
    JointState con una marca de tiempo y se relee TF hasta que la marca
    de la transformada alcanza o supera a la publicada. Recien entonces
    se sabe que la pose leida corresponde a la q enviada.

Metrica

    error posicional   ||p_tf - p_ana||                    [m]
    error angular      ||R_ana^T . R_tf - I||_F            [adimensional]

    El error angular usa que R_ana^T . R_tf = I si y solo si ambas
    rotaciones coinciden, porque R^-1 = R^T en SO(3). La norma de
    Frobenius de la diferencia con la identidad mide cuanto se aparta.

Uso
    ros2 launch irb120_description display.launch.py gui:=false
    pkill -f joint_state_publisher
    python3 validate_tf.py            (1000 muestras por defecto)
    python3 validate_tf.py 200        (numero de muestras a medida)

Grupo VEVAS
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from fk_irb120 import forward_kinematics, JOINT_LIMITS

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
UMBRAL = 1e-6          # exigencia del hito, en metros


def quat_to_matrix(x, y, z, w):
    """
    Cuaternion unitario -> matriz de rotacion.

    Formula de Euler-Rodrigues expandida. Se implementa a mano en vez de
    usar una libreria porque forma parte de lo que hay que poder
    defender. Se normaliza antes por seguridad numerica.
    """
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def stamp_a_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Validador(Node):

    def __init__(self, n_muestras):
        super().__init__("validador_gemelo_digital")
        self.n = n_muestras
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def revisar_competencia(self):
        """
        Avisa si otro nodo tambien publica en /joint_states.

        Si joint_state_publisher esta vivo, sobreescribe las
        configuraciones de este script con q = 0 y los resultados salen
        sin sentido. Se cuenta el propio publicador, asi que mas de uno
        es senal de conflicto.
        """
        n = self.count_publishers("/joint_states")
        if n > 1:
            print(f"  AVISO: hay {n} publicadores en /joint_states.")
            print("  Otro nodo compite por el control de las juntas.")
            print("  Cierrelo antes de validar:  pkill -f joint_state_publisher")
            print()
        return n

    def publicar(self, q):
        """Publica q sellada con la hora actual y devuelve esa marca."""
        msg = JointState()
        ahora = self.get_clock().now().to_msg()
        msg.header.stamp = ahora
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub.publish(msg)
        return stamp_a_ns(ahora)

    def leer_tf_sincronizado(self, t_pub_ns, timeout=2.0):
        """
        Devuelve (p, R) de base_link -> tool0 correspondiente a la
        configuracion publicada en t_pub_ns.

        Relee hasta que la marca de tiempo de la transformada alcanza a
        la del JointState enviado. Asi se garantiza que la pose leida no
        es la de la configuracion anterior.
        """
        fin = time.time() + timeout
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.005)
            try:
                t = self.buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time())
            except Exception:
                continue

            if stamp_a_ns(t.header.stamp) < t_pub_ns:
                continue          # todavia es la pose vieja

            tr = t.transform.translation
            ro = t.transform.rotation
            return (np.array([tr.x, tr.y, tr.z]),
                    quat_to_matrix(ro.x, ro.y, ro.z, ro.w))
        return None

    def correr(self):
        rng = np.random.default_rng(7)
        err_p, err_r = [], []
        fallos = 0

        print(f"\n   Validacion TF vs analitica   {self.n} configuraciones\n")

        self.revisar_competencia()

        # Calentamiento: el listener necesita acumular el arbol antes de
        # que cualquier lookup pueda tener exito.
        for _ in range(20):
            self.publicar(np.zeros(6))
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

        for k in range(self.n):
            q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            t_pub = self.publicar(q)

            lectura = self.leer_tf_sincronizado(t_pub)
            if lectura is None:
                fallos += 1
                continue
            p_tf, R_tf = lectura

            T = forward_kinematics(q)
            p_ana, R_ana = T[:3, 3], T[:3, :3]

            err_p.append(np.linalg.norm(p_tf - p_ana))
            err_r.append(np.linalg.norm(R_ana.T @ R_tf - np.eye(3), "fro"))

            if (k + 1) % 100 == 0:
                print(f"  {k + 1:5d} / {self.n}   "
                      f"peor pos {max(err_p):.3e} m")

        if not err_p:
            print("\n  No se obtuvo ninguna lectura de TF.")
            print("  Verifique que display.launch.py este corriendo.\n")
            return False

        ep = np.array(err_p)
        er = np.array(err_r)

        print(f"\nResultados sobre {len(ep)} muestras validas")
        print(f"  Error posicional   medio  {ep.mean():.3e} m")
        print(f"  Error posicional   maximo {ep.max():.3e} m")
        print(f"  Error angular      maximo {er.max():.3e}")
        if fallos:
            print(f"  Lecturas perdidas por timeout: {fallos}")

        ok = ep.max() < UMBRAL
        print(f"\n  Umbral exigido por el hito: {UMBRAL:.0e} m")
        if ok:
            print(f"  Margen: {UMBRAL / max(ep.max(), 1e-300):.1e}x "
                  f"por debajo del umbral")
        else:
            print(f"  Excede el umbral por un factor de "
                  f"{ep.max() / UMBRAL:.1e}x")
        print("  RESULTADO:", "APROBADO" if ok else "NO APROBADO")
        print()
        return ok


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    rclpy.init()
    nodo = Validador(n)
    try:
        ok = nodo.correr()
    finally:
        nodo.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
