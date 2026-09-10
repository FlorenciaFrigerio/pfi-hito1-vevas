#!/usr/bin/env python3
"""
Lanza el gemelo digital del ABB IRB 120 en RViz.

Cadena de responsabilidad
    urdf              descripcion, va al parametro robot_description
    joint_state_pub   publica /joint_states con los 6 valores q_i
    robot_state_pub   compone Origin_i . Rz(q_i) y publica /tf
    rviz2             consume /tf y dibuja marcos y mallas

Ningun nodo conoce la convencion D-H: solo recorren el arbol XML.

Grupo VEVAS
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('irb120_description')

    # Permite elegir cual de los dos URDF se carga, para demostrar en la
    # defensa que ambos producen exactamente el mismo arbol TF.
    modelo = LaunchConfiguration('modelo')
    gui = LaunchConfiguration('gui')

    ruta_urdf = PythonExpression([
        "'", os.path.join(pkg, 'urdf'), "/' + '", modelo, "' + '.urdf'"
    ])

    # El URDF se lee como texto plano y se entrega como parametro string.
    # ParameterValue con value_type=str evita que ROS intente inferir el
    # tipo y convierta el XML en algo que no es.
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue

    robot_description = ParameterValue(
        Command(['cat ', ruta_urdf]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'modelo', default_value='irb120',
            description='irb120 (generado) o irb120_manual (escrito a mano)'
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true abre los sliders, false publica q = 0'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),


         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'irb120.rviz')],
            output='screen',
        ),
    ])
