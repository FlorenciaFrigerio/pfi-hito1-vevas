# Guía paso a paso — Hito 2

**Grupo VEVAS** · Motor Cinemático Inverso (Pieper) y Perfiles LSPB

---

## Qué exige el Hito 2 (fuentes oficiales)

De la **Guía Oficial del PFI**:

> **Entregable:** Nodo de ROS en Python que reciba una pose, ejecute Pieper y
> genere una trayectoria LSPB.
>
> **Pasaporte de Aprobación:** Demostración en Jupyter Notebook de la
> divergencia analítica de velocidades y torques en singularidad
> (q5 = 0 rad).

Son **dos entregables distintos**: el nodo de ROS y el notebook. No basta con
uno de los dos.

De la **guía W09-S01**, la evidencia requerida por componente:

| Componente | Evidencia exigida |
|---|---|
| IK Analítica (Pieper) | 8 ramas; lógica de filtrado; validación obligatoria por FK |
| Jacobiano | implementación; **al menos un caso de validación manual**; interpretación física |
| Singularidades | experimento documentado; rango/determinante; movilidad perdida; trazado cerca de la singularidad |
| LSPB | ecuaciones q(t), q̇(t), q̈(t); justificación de parámetros; continuidad y límites |
| Dinámica | modelo; τ(t) sobre la trayectoria; interpretación |
| Notebook | ecuaciones, código limpio, **unidades explícitas**, gráficas, discusión y conclusiones |

Del **esqueleto oficial W09-S01**, las 9 secciones obligatorias del notebook —
respetadas exactamente en `Hito2_VEVAS.ipynb`.

Del **handoff de dinámica W09-S01**:

> NO se debe intentar realizar una derivación simbólica de los 6 grados de
> libertad del robot IRB120, a menos que se suministre explícitamente
> información geométrica inercial del fabricante.

Por eso se usa un 2R planar, declarado como tal.

---

## PASO 0 — Abrir la máquina virtual

Arrancá la VM de Ubuntu 24.04 y abrí una terminal con `Ctrl+Alt+T`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/irb120_description
git status
```

Debe decir `working tree clean`. Si hay cambios sin commitear del Hito 1,
resolvélos antes de seguir.

---

## PASO 1 — Rama de trabajo

El PFI exige el modelo de ramas `main` / `dev` / `feature/*`:

```bash
git checkout dev
git pull origin dev 2>/dev/null || true
git checkout -b feature/hito2-pieper-lspb
```

---

## PASO 2 — Dependencias

```bash
sudo apt install -y python3-sympy python3-matplotlib jupyter
```

Verificá:

```bash
python3 -c "import numpy, sympy, matplotlib; print('ok')"
jupyter --version
```

Si `jupyter` no aparece, instalalo con pip:

```bash
pip install --break-system-packages notebook
```

---

## PASO 3 — Colocar los archivos

Descargá los archivos y movelos a su sitio:

```bash
cd ~/ros2_ws/src/irb120_description

mv ~/Downloads/pieper.py        scripts/
mv ~/Downloads/jacobiano.py     scripts/
mv ~/Downloads/lspb.py          scripts/
mv ~/Downloads/dinamica.py      scripts/
mv ~/Downloads/singularidad.py  scripts/
mv ~/Downloads/graficas.py      scripts/
mv ~/Downloads/nodo_hito2.py    scripts/
chmod +x scripts/*.py

mkdir -p notebooks
mv ~/Downloads/Hito2_VEVAS.ipynb notebooks/
```

Los módulos van **junto a `fk_irb120.py`** del Hito 1, porque importan de ahí
la tabla D-H. Una sola fuente de verdad: si cambia un parámetro, todo el
pipeline se actualiza solo.

Verificá la estructura:

```bash
ls scripts/ notebooks/
```

---

## PASO 4 — Probar cada módulo por separado

**En orden.** Si uno falla, no sigas al siguiente.

```bash
cd ~/ros2_ws/src/irb120_description/scripts
```

### 4.1 Pieper

```bash
python3 pieper.py
```

Esperado:

```
  ramas validas por pose (media)  8.00
  poses sin solucion              0
  error posicional maximo         9.103e-15 m
  RESULTADO: OK
```

### 4.2 Jacobiano

```bash
python3 jacobiano.py
```

Esperado: discrepancia `3.249e-08` contra diferencias finitas, validación
manual `2.082e-18`, y el 2R planar coincidiendo con `det = a1·a2·sin(q2)`.

### 4.3 LSPB

```bash
python3 lspb.py
```

Esperado: `T = 1.3000 s` común, `qd nula en extremos True`.

### 4.4 Dinámica

```bash
python3 dinamica.py
```

Esperado: asimetría de M exactamente `0.000e+00`, antisimetría `3.911e-11`,
deriva de energía `4.303e-04`, `RESULTADO: OK`.

### 4.5 El pasaporte

```bash
python3 singularidad.py
```

**Éste es el que define el hito.** Esperado:

```
  Ley de escalado ajustada:  ||qd|| ~ q5^(-1.000)     CONFIRMADO
  Ley de escalado ajustada:  tau ~ q5^(-2.000)        CONFIRMADO
```

### 4.6 Gráficas

```bash
python3 graficas.py
ls *.pdf
```

Genera `fig_lspb.pdf`, `fig_torque.pdf`, `fig_divergencia.pdf`,
`fig_manipulabilidad.pdf`.

---

## PASO 5 — Correr el notebook

```bash
cd ~/ros2_ws/src/irb120_description/notebooks
jupyter notebook Hito2_VEVAS.ipynb
```

Se abre el navegador. **Antes de ejecutar**, decile a Jupyter dónde están los
módulos: ya está resuelto porque el notebook está en `notebooks/` y los
módulos en `scripts/`. Si da `ModuleNotFoundError`, agregá como primera celda:

```python
import sys; sys.path.insert(0, "../scripts")
```

Ejecutá todo con **Cell → Run All** (o `Kernel → Restart & Run All`).

Tiene que terminar en el bloque `HITO 2 - RESUMEN DE VALIDACION` sin errores.

Tarda alrededor de un minuto: la sección 3 resuelve 2000 poses y la sección 7
integra 200 000 pasos para verificar conservación de energía.

---

## PASO 6 — Compilar y probar el nodo de ROS

Agregá el nodo al `CMakeLists.txt`. Abrilo:

```bash
nano ~/ros2_ws/src/irb120_description/CMakeLists.txt
```

El bloque que instala `scripts/` ya existe del Hito 1 con
`USE_SOURCE_PERMISSIONS`, así que el nodo queda instalado automáticamente.
Verificá que esté:

```cmake
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
```

Compilá:

```bash
cd ~/ros2_ws
colcon build --packages-select irb120_description
source install/setup.bash
```

### Probar el nodo

**Terminal A** — el gemelo digital del Hito 1, sin sliders:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch irb120_description display.launch.py gui:=false
```

**Terminal B** — el nodo del Hito 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/irb120_description/scripts
python3 nodo_hito2.py
```

**Terminal C** — publicar una pose objetivo:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /pose_objetivo geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'},
  pose: {position: {x: 0.35, y: 0.10, z: 0.55},
         orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

En la Terminal B tenés que ver algo así:

```
ACEPTADA  2/8 ramas admisibles  |  elegida: codo arriba / muneca no
invertida / hombro frontal  |  dE = 3.2e-16 m, dR = 1.1e-15  |  T = 0.84 s
```

Y en RViz el robot se mueve siguiendo el perfil LSPB.

### Probar el rechazo de poses inalcanzables

```bash
ros2 topic pub --once /pose_objetivo geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'},
  pose: {position: {x: 2.0, y: 2.0, z: 2.0},
         orientation: {w: 1.0}}}"
```

Esperado: `POSE INALCANZABLE ... 8 ramas calculadas, 0 admisibles`.
El robot **no se mueve**. Eso es importante mostrarlo en la defensa: el sistema
falla de forma segura.

### Ver la trayectoria publicada

```bash
ros2 topic echo /trayectoria_hito2 --once | head -40
ros2 topic echo /diagnostico_hito2
```

---

## PASO 7 — Commits

El PFI penaliza los commits masivos. Hacelos por capa, a medida que avanzás:

```bash
cd ~/ros2_ws/src/irb120_description

git add scripts/pieper.py
git commit -m "feat(ik): resolvedor analitico de Pieper con 8 ramas

Desacoplamiento por muneca esferica. Eslabon equivalente
k = sqrt(a3^2 + d4^2) reduce el subproblema de posicion a un 2R planar.
Filtrado en tres etapas (numerico, cinematico, limites) y seleccion por
minimo desplazamiento ponderado.
Verificado: 8.00 ramas por pose, error 9.10e-15 m sobre 2000 poses."

git add scripts/jacobiano.py
git commit -m "feat(jacobian): jacobiano geometrico y diagnostico de singularidad

Construccion por columnas Jv = z x (pe - p), Jw = z.
Validacion manual en q=0 (2.08e-18) y contra diferencias finitas (3.25e-08).
Contraste con el 2R planar analitico det = a1 a2 sin(q2)."

git add scripts/lspb.py
git commit -m "feat(trajectory): perfiles LSPB con sincronizacion multi-junta

Dos parametrizaciones: (V, A) y (tf, qddc) de la hoja de trabajo W09-S01,
con condicion de factibilidad |qddc| >= 4|h|/tf^2.
Reproduce el ejemplo de clase: tc = 1 s, V = 1 rad/s, T = 3 s."

git add scripts/dinamica.py
git commit -m "feat(dynamics): modelo 2R planar y evaluacion de torque

tau = M qdd + C qd + g sobre el brazo planar de las juntas 2 y 3, segun
el alcance fijado en el handoff W09-S01 (no se derivan los 6 GDL).
Validado por simetria de M, antisimetria de Mdot-2C y conservacion de
energia (deriva 4.3e-04)."

git add scripts/singularidad.py
git commit -m "feat(singularity): pasaporte del hito, divergencia en q5 = 0

Demuestra las dos leyes de escalado por ajuste log-log:
  ||qd|| ~ q5^(-1.000)   divergencia hiperbolica de velocidad
  tau    ~ q5^(-2.000)   divergencia cuadratica de torque
Incluye el contraejemplo singularidad != torque alto."

git add scripts/graficas.py
git commit -m "feat(plots): figuras del hito con paleta validada para daltonismo"

git add scripts/nodo_hito2.py
git commit -m "feat(ros): nodo Hito 2, pose -> Pieper -> LSPB

Suscribe /pose_objetivo, resuelve por Pieper, filtra ramas y publica
/joint_states a 100 Hz y /trayectoria_hito2.
Alerta de proximidad a singularidad antes de mover."

git add notebooks/Hito2_VEVAS.ipynb
git commit -m "docs(notebook): entrega computacional del Hito 2

Nueve secciones segun el esqueleto oficial W09-S01."

git push -u origin feature/hito2-pieper-lspb
```

Después mergeá a `dev` y etiquetá:

```bash
git checkout dev
git merge feature/hito2-pieper-lspb
git push origin dev

git checkout main
git merge dev
git tag -a v2.0-hito2 -m "Hito 2: Pieper + LSPB + dinamica. Pasaporte: qd ~ q5^-1, tau ~ q5^-2"
git push origin main --tags
```

---

## PASO 8 — Video demostrativo

Tres tomas, 3 a 4 minutos.

**Toma 1 — el nodo funcionando.** Terminal B con el nodo, Terminal C
publicando poses, RViz al costado. Mostrá una pose válida (se mueve) y una
inalcanzable (no se mueve, avisa).

**Toma 2 — el pasaporte.** Corré `python3 singularidad.py` y dejá que
termine. Señalá las dos pendientes `-1.000` y `-2.000`.

**Toma 3 — el notebook.** `Kernel → Restart & Run All` y mostrá que llega al
resumen final sin errores.

Grabar: `Ctrl+Alt+Shift+R` en Ubuntu, o instalá OBS si querés narrar.

---

## Preguntas que te van a hacer, y la respuesta

**¿Por qué 8 soluciones y no una?**
Tres signos independientes: hombro (q1 o q1+π), codo (±arccos) y muñeca
(±arccos). 2³ = 8. No son artificios algebraicos: cada una es una postura
física distinta que alcanza la misma pose.

**¿Por qué `atan2` y no `arctan`?**
`arctan(y/x)` devuelve ángulos en (−π/2, π/2) y pierde el cuadrante: no
distingue (1,1) de (−1,−1). Con `arctan` la mitad de las ramas sería
inalcanzable.

**¿Cómo eligen una rama entre las 8?**
Mínimo desplazamiento articular ponderado, con pesos [4,3,2,1,1,0.5]. Los
pesos penalizan más las juntas proximales: girar la base arrastra todo el
brazo. Sin pesos, el criterio elegiría ramas que hacen girar media vuelta la
base para ahorrar grados en la muñeca.

**¿Por qué el torque diverge si usted dijo que singularidad ≠ torque alto?**
Ésta es la pregunta trampa. La divergencia es **indirecta**: la singularidad
hace divergir q̇ por cinemática diferencial inversa, q̈ diverge como derivada
de q̇, y recién entonces τ = M q̈ + ... hereda la divergencia. La singularidad
no produce torque: produce velocidad articular infinita. Y al revés: la
sección 5.e muestra una postura regular con 24 N·m de torque gravitatorio, sin
singularidad alguna. Son fenómenos independientes.

**¿Por qué el pico de torque no está donde la aceleración es máxima?**
Porque la gravedad domina al término inercial por un factor 6,9 en esta
trayectoria. Durante el crucero, con q̈ = 0, el motor sigue entregando casi
todo el torque sólo para sostener el brazo.

**¿Por qué un modelo 2R y no los 6 GDL?**
Por indicación expresa del handoff W09-S01: ABB no publica los tensores de
inercia, así que una matriz M de 6×6 sería inventada. El 2R captura el
subsistema que soporta el par gravitatorio dominante. Las validaciones que
aplicamos (simetría de M, antisimetría de Ṁ−2C, conservación de energía) son
propiedades estructurales que se cumplen con cualquier parámetro, así que
validan el modelo aunque las inercias sean estimadas.

**¿Por qué la manipulabilidad arranca en cero?**
Porque la posición de reposo q = 0 tiene q5 = 0: **el HOME del robot es la
singularidad de muñeca**. Lo descubrimos al graficar. Tiene consecuencia de
diseño para el Hito 4: hay que salir de la vecindad singular bajo control
articular antes de habilitar control cartesiano.

**¿Por qué el LSPB puede atravesar la singularidad sin problema?**
Porque LSPB interpola en el espacio **articular** y nunca invierte el
Jacobiano. La divergencia aparece sólo al resolver q̇ = J⁻¹ẋ, es decir bajo
control cartesiano.

---

## Lista de verificación antes de entregar

- [ ] Los 6 módulos corren sin error
- [ ] `singularidad.py` muestra las pendientes −1.000 y −2.000
- [ ] El notebook llega al resumen final con `Restart & Run All`
- [ ] El nodo acepta una pose válida y rechaza una inalcanzable
- [ ] Commits por capa, no uno masivo
- [ ] Tag `v2.0-hito2` apuntando al commit final
- [ ] Video subido y enlazado en el README
- [ ] Repartidas las secciones del notebook entre los cuatro
