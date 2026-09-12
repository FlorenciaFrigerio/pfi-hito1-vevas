# irb120_description

Gemelo digital cinemático del manipulador **ABB IRB 120** (6 GDL).

Descripción URDF generada programáticamente a partir de la tabla
Denavit-Hartenberg y validada contra la cinemática directa analítica.

**Grupo VEVAS** · Hito 1 del Proyecto Final Integrador
Robótica IMT-342 · Gestión II-2026

**Material demostrativo:** [carpeta en Drive](https://drive.google.com/drive/folders/1uHc6FJT6_jHLQa7Mz7QFIGZA61dkwgJ8?usp=sharing)
— videos de la visualización en RViz, la validación TF y el historial del repositorio.

---

## Web interactiva del brazo ABB

La carpeta [web/](web/) contiene el explorador técnico del **ABB IRB 120**:
modelo 3D, selección de conjuntos, vista de articulaciones y celda de trabajo.

## Resultado

| Métrica | Valor | Umbral del hito |
|---|---|---|
| Error posicional máximo (TF vs. analítica) | 2,459 × 10⁻¹⁰ m | 1 × 10⁻⁶ m |
| Error posicional medio | 1,498 × 10⁻¹⁰ m | — |
| Error angular máximo | 1,092 × 10⁻⁹ | — |
| Muestras válidas | 501 de 1000 | — |
| Margen respecto al umbral | 4,1 × 10³ veces por debajo | — |

Verificación interna del generador (sin pasar por ROS): **3,68 × 10⁻¹⁶ m**.

La diferencia entre ambas cifras no es error del modelo: TF transporta las
rotaciones como cuaterniones en `float32`, cuyo épsilon es 1,2 × 10⁻⁷. El
modelo es exacto hasta el épsilon de `float64`; lo que se mide vía TF es la
precisión del transporte de ROS.

---

## Tabla Denavit-Hartenberg

Parametrización de Truc & Lam (2020), contrastada con las cotas del manual
técnico oficial de ABB.

| i | θᵢ | dᵢ [m] | aᵢ [m] | αᵢ | Interpretación |
|---|---|---|---|---|---|
| 1 | q₁ | 0,290 | 0 | −π/2 | Altura de la columna base |
| 2 | q₂ − π/2 | 0 | 0,270 | 0 | Brazo, ejes paralelos |
| 3 | q₃ | 0 | 0,070 | −π/2 | Codo, offset lateral corto |
| 4 | q₄ | 0,302 | 0 | +π/2 | Antebrazo hacia la muñeca |
| 5 | q₅ | 0 | 0 | −π/2 | Ejes concurrentes (a = d = 0) |
| 6 | q₆ | 0,072 | 0 | 0 | Brida de salida (tool0) |

### Verificación de cierre en q = 0

Sin multiplicar una sola matriz, las cotas del manual predicen la pose:

```
x = d₄ + d₆      = 0,302 + 0,072         = 0,374 m
z = d₁ + a₂ + a₃ = 0,290 + 0,270 + 0,070 = 0,630 m
```

### El offset θ₂ = q₂ − π/2

Hay dos ceros distintos y no coinciden. El cero mecánico de ABB corresponde
al brazo vertical (postura de calibración del fabricante). El cero de D-H
exige que x₁ y x₂ sean paralelos, lo que ocurre con el brazo horizontal.
El offset reconcilia ambos y se codifica una sola vez dentro de la
transformación, nunca en el `joint_state`.

---

## Conversión D-H → URDF

URDF impone `T_padre→hijo = Origin · Rot(eje, q)` con `Origin` constante,
pero en D-H la variable de junta va al principio. Se resuelve factorizando:

```
Aᵢ = Rz(qᵢ + offᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)
   = Rz(offᵢ) · Rz(qᵢ) · Bᵢ        con  Bᵢ = Tz(dᵢ)·Tx(aᵢ)·Rx(αᵢ)
```

porque `Rz(a)·Rz(b) = Rz(a+b)`: las rotaciones en torno al mismo eje
conmutan. Reordenando la cadena, cada junta URDF recibe

```
Originᵢ = Bᵢ₋₁ · Rz(offᵢ)        con  B₀ = I
```

y una junta fija final con `B₆` cierra en la brida. Como `Bᵢ` es traslación
pura seguida de un giro en x, se reduce a `xyz="aᵢ 0 dᵢ"` y `rpy="αᵢ 0 0"`.

Ningún origen fue ajustado visualmente en RViz.

---

## Estructura

```
irb120_description/
├── urdf/
│   ├── irb120_manual.urdf     escrito a mano, procedencia documentada
│   └── irb120.urdf            generado por script (no editar a mano)
├── scripts/
│   ├── fk_irb120.py           cinemática directa, sólo NumPy
│   ├── generate_urdf.py       derivación D-H → URDF + verificación offline
│   ├── compare_urdf.py        equivalencia manual vs. derivado
│   └── validate_tf.py         validación contra el árbol TF de ROS
├── launch/display.launch.py
├── rviz/irb120.rviz
├── meshes/
└── docs/validacion_hito1.txt  evidencia numérica
```

Se mantienen **dos** URDF de forma deliberada: el manual documenta el
razonamiento detrás de cada número, el derivado lo demuestra. `compare_urdf.py`
verifica que sean el mismo robot.

---

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3 con NumPy

El BOM del proyecto especifica Ubuntu 24.04 con ROS Noetic o Humble. Esa
combinación no existe: Noetic sólo soporta Ubuntu 20.04 y Humble sólo 22.04.
Se respetó el sistema operativo indicado y se adoptó Jazzy, la distribución
LTS oficial para 24.04.

```bash
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2
```

---

## Reproducir los resultados

```bash
# Compilar
cd ~/ros2_ws
colcon build --packages-select irb120_description
source install/setup.bash
```

```bash
# 1. Cinemática analítica: p(q=0) = [0.374, 0, 0.630] m
cd src/irb120_description/scripts
python3 fk_irb120.py
```

```bash
# 2. Generar el URDF y verificar la cadena offline
python3 generate_urdf.py

# 3. Comprobar equivalencia manual vs. derivado
python3 compare_urdf.py

# 4. Validar la sintaxis XML y el árbol
check_urdf ../urdf/irb120.urdf
```

```bash
# 5. Visualización con sliders
ros2 launch irb120_description display.launch.py

# Lectura numérica de la pose, en otra terminal
ros2 run tf2_ros tf2_echo base_link tool0
```

```bash
# 6. Validación TF vs. analítica  (sin GUI: evita conflicto de publicadores)
ros2 launch irb120_description display.launch.py gui:=false

# En otra terminal
cd src/irb120_description/scripts
python3 validate_tf.py 1000
```

El argumento `modelo` permite cargar cualquiera de los dos URDF y comprobar
que producen el mismo árbol TF:

```bash
ros2 launch irb120_description display.launch.py modelo:=irb120_manual
```

---

## Notas de implementación

**Sincronización en la validación.** Publicar una configuración y leer TF
tras una espera fija devuelve la transformada de la configuración
*anterior*, porque `robot_state_publisher` aún no procesó el mensaje. El
error resultante era superior a 1 m. Se resolvió sellando cada `JointState`
con una marca de tiempo y releyendo TF hasta que la marca de la transformada
la alcanza.

**Publicadores en conflicto.** `joint_state_publisher` reescribe las
configuraciones del validador con q = 0. Por eso `gui:=false` no levanta
ningún publicador de juntas, dejando el canal libre.

**Gimbal lock en la conversión rpy.** El origen de `joint_2` requiere
convertir `Rx(−π/2)·Rz(−π/2)`, cuya matriz tiene `R[2][0] = 1`. Ahí
`pitch = −π/2` y roll y yaw dejan de ser independientes. `matrix_to_rpy()`
implementa la rama degenerada explícitamente.

**Instalación de directorios.** Si `CMakeLists.txt` no instala `urdf/`,
`meshes/` y `launch/` en `share/`, las rutas `package://` no resuelven: el
árbol TF sale correcto pero RViz no dibuja geometría. Es un fallo silencioso.

---

## Trabajo siguiente

- Mallas STL de los eslabones
- Pinza neumática (diseño propio en SolidWorks)
- Tags `<inertial>` para habilitar Gazebo
- Hito 2: cinemática inversa por desacoplamiento de Pieper

---

## Licencia

MIT. Ver [LICENSE](LICENSE).
