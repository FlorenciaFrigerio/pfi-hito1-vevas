#!/usr/bin/env python3
"""
Nodo ROS 2 del Hito 2.

    "Entregable: Nodo de ROS en Python que reciba una pose, ejecute
     Pieper y genere una trayectoria LSPB."
                                    -- Guia Oficial del PFI, Hito 2

Arquitectura

    /pose_objetivo  (geometry_msgs/PoseStamped)
            |
            v
    [ Pieper ]  ->  8 ramas analiticas
            |
            v
    [ filtrado ]  ->  FK + limites articulares
            |
            v
    [ seleccion ] ->  minimo desplazamiento ponderado
            |
            v
    [ LSPB ]  ->  q(t), qd(t), qdd(t) sincronizados
            |
            +--> /joint_states          (sensor_msgs/JointState)   a 100 Hz
            +--> /trayectoria_hito2     (trajectory_msgs/JointTrajectory)
            +--> /diagnostico_hito2     (std_msgs/String)

No se usa MoveIt! ni ningun resolvedor automatico: regla anti-caja-negra
del PFI. Tampoco hay coordenadas pregrabadas; la pose llega por el topico.

Uso

    ros2 run irb120_description nodo_hito2.py

    # en otra terminal, publicar una pose objetivo:
    ros2 topic pub --once /pose_objetivo geometry_msgs/msg/PoseStamped \\
      "{header: {frame_id: 'base_link'},
        pose: {position: {x: 0.35, y: 0.10, z: 0.55},
               orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

Grupo VEVAS
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from fk_irb120 import JOINT_LIMITS, forward_kinematics
from pieper import resolver, etiquetar, error_pose
from lspb import LSPBSincronizado
from jacobiano import jacobiano, medida_manipulabilidad

NOMBRES = [f"joint_{i}" for i in range(1, 7)]

# Limites de marcha. Se toman por debajo del datasheet a proposito: el
# robot admite mas, pero el proyecto exige trayectorias suaves y un
# margen frente a la saturacion de los actuadores.
V_MAX = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])      # rad/s
A_MAX = np.array([4.0, 4.0, 5.0, 6.0, 6.0, 8.0])      # rad/s^2

FRECUENCIA = 100.0      # Hz de publicacion de /joint_states
UMBRAL_W = 1e-3         # manipulabilidad por debajo de la cual se avisa


def quat_a_matriz(x, y, z, w):
    """Cuaternion unitario -> matriz de rotacion (Euler-Rodrigues)."""
    n = np.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        raise ValueError("cuaternion nulo")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class NodoHito2(Node):

    def __init__(self):
        super().__init__("nodo_hito2")

        self.q_actual = np.zeros(6)
        self.traj = None
        self.t0 = None

        self.sub = self.create_subscription(
            PoseStamped, "/pose_objetivo", self.cb_pose, 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_traj = self.create_publisher(
            JointTrajectory, "/trayectoria_hito2", 10)
        self.pub_diag = self.create_publisher(
            String, "/diagnostico_hito2", 10)

        self.timer = self.create_timer(1.0/FRECUENCIA, self.cb_timer)

        self.get_logger().info(
            "Nodo Hito 2 activo. Esperando pose en /pose_objetivo")
        self.publicar_estado(self.q_actual)

    # ------------------------------------------------------------------
    def cb_pose(self, msg):
        """Pose recibida -> Pieper -> filtrado -> LSPB."""
        p = msg.pose.position
        o = msg.pose.orientation
        try:
            R = quat_a_matriz(o.x, o.y, o.z, o.w)
        except ValueError as e:
            self.avisar(f"RECHAZADA: {e}")
            return

        T_d = np.eye(4)
        T_d[:3, :3] = R
        T_d[:3, 3] = [p.x, p.y, p.z]

        q_obj, admisibles, rechazadas = resolver(T_d, q_actual=self.q_actual)

        if q_obj is None:
            motivos = {}
            for _, m in rechazadas:
                clave = m.split(":")[0]
                motivos[clave] = motivos.get(clave, 0) + 1
            detalle = ", ".join(f"{k} x{v}" for k, v in motivos.items())
            self.avisar(
                f"POSE INALCANZABLE  p=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]  "
                f"8 ramas calculadas, 0 admisibles ({detalle})")
            return

        ep, eR = error_pose(q_obj, T_d)

        # Advertencia de singularidad ANTES de mover: si la pose objetivo
        # esta cerca de q5 = 0, la cinematica diferencial inversa exigiria
        # velocidades articulares desmedidas.
        w = medida_manipulabilidad(q_obj)
        aviso_sing = ""
        if abs(q_obj[4]) < 0.05 or w < UMBRAL_W:
            aviso_sing = (f"  [!] CERCA DE SINGULARIDAD: q5 = {q_obj[4]:+.4f} "
                          f"rad, w = {w:.3e}")

        self.traj = LSPBSincronizado(self.q_actual, q_obj, V_MAX, A_MAX)
        self.t0 = self.get_clock().now()

        self.avisar(
            f"ACEPTADA  {len(admisibles)}/8 ramas admisibles  |  "
            f"elegida: {etiquetar(q_obj)}  |  "
            f"dE = {ep:.2e} m, dR = {eR:.2e}  |  "
            f"T = {self.traj.T:.3f} s{aviso_sing}")

        self.publicar_trayectoria(q_obj)

    # ------------------------------------------------------------------
    def cb_timer(self):
        """Muestrea el perfil LSPB y publica el estado articular."""
        if self.traj is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if t >= self.traj.T:
            t = self.traj.T
            q, _, _ = self.traj(t)
            self.q_actual = q[0]
            self.publicar_estado(self.q_actual)
            self.avisar(f"TRAYECTORIA COMPLETA  q_final = "
                        f"{np.round(self.q_actual, 4).tolist()}")
            self.traj = None
            return

        q, qd, _ = self.traj(t)
        self.q_actual = q[0]
        self.publicar_estado(q[0], qd[0])

    # ------------------------------------------------------------------
    def publicar_estado(self, q, qd=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NOMBRES
        msg.position = [float(v) for v in q]
        if qd is not None:
            msg.velocity = [float(v) for v in qd]
        self.pub_js.publish(msg)

    def publicar_trayectoria(self, q_obj, n=50):
        """Publica la trayectoria completa, para quien quiera inspeccionarla."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = NOMBRES
        for t in np.linspace(0.0, self.traj.T, n):
            q, qd, qdd = self.traj(t)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q[0]]
            pt.velocities = [float(v) for v in qd[0]]
            pt.accelerations = [float(v) for v in qdd[0]]
            pt.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(pt)
        self.pub_traj.publish(msg)

    def avisar(self, texto):
        self.get_logger().info(texto)
        self.pub_diag.publish(String(data=texto))


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoHito2()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
