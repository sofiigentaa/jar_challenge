# Donatello: versión experimental de rescate

Esta rama reemplaza el nodo vacío del template. **No está validada todavía con ROS 2, Gazebo ni un robot físico.** No es una entrega oficial ni una garantía de puntaje. La rama principal se conserva.

## Cambios

- AMCL con modelo omnidireccional localiza el robot contra el `/map` del juez; no se usa pose real del simulador. La referencia inicial aproximada es el origen que establece el reglamento.
- Planificación de cobertura sobre el mapa recibido, obstáculos inflados por el radio y margen del robot, y capa temporal de obstáculos observados por LiDAR. Desconocido y bordes se consideran bloqueados. Diagonales sin cortar esquinas.
- RGB segmentado en rojo y retornos LiDAR proyectados a la cámara con calibración y TF del instante del escaneo. Solo observaciones próximas en el tiempo, con varias confirmaciones y dispersión limitada, generan un reporte.
- Registro espacial con ancla inmutable tras publicar, límite N y bloqueo de asociaciones ambiguas. No se retransmiten reportes. Esto evita repeticiones dentro del radio de asociación; **no garantiza identidad ante errores grandes de localización**.
- Velocidad máxima inicial de 0,12 m/s, con límite por distancia de frenado y latencia. Freno ante sensores viejos, cobertura inválida, cámara ausente, TF faltante o localización incierta.
- Regresa al punto inicial estimado al completar N reportes o a los 450 segundos. Publica `/done` una sola vez al permanecer cerca del inicio estimado. Enviar N reportes no confirma que N hayan puntuado.

**Los 25 cm del reglamento son el error entre la coordenada reportada y el centro real de la víctima. No exigen llevar el robot a 25 cm de la caja.** Los 200 puntos requieren las N víctimas con puntaje antes de 300 s; el regreso puede ser posterior.

## Ejecutar en tu contenedor de Linux Mint

El entorno ROS y el simulador existentes deben estar instalados. En el host:

```bash
docker exec -it donatello-gui bash
```

Descargá `Donatello_rescate_prueba.zip` y extraelo en `~/Descargas`. Desde la terminal del host, copiá la carpeta nueva al contenedor (sin reemplazar tu workspace del simulador):

```bash
cd ~/Descargas
unzip Donatello_rescate_prueba.zip
docker cp jar_rescate_prueba donatello-gui:/root/
docker exec -it donatello-gui bash
source /opt/ros/humble/setup.bash
source /root/donatello_ws/install/setup.bash
cd /root/jar_rescate_prueba
```

No hay rama publicada: GitHub rechazó la escritura de la conexión (403). El ZIP contiene el template completo con los cambios y conserva `tools/` sin modificar.

Si faltan dependencias, dentro del contenedor:

```bash
apt-get update
apt-get install -y ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager ros-humble-cv-bridge ros-humble-tf2-ros python3-numpy python3-scipy python3-opencv python3-pytest
```

Cerrá otros controladores y teleoperadores antes de probar. No ejecutes dos simuladores simultáneamente.

Terminal 1 dentro del contenedor (si el simulador no está abierto):

```bash
source /opt/ros/humble/setup.bash
source /root/donatello_ws/install/setup.bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py world:=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)/worlds/maze_1_6x5_victimas.world
```

Terminal 2 dentro del contenedor, iniciar software. El ejemplo usa 3 víctimas; verificá la cantidad que informa el juez para el mundo elegido:

```bash
source /opt/ros/humble/setup.bash
source /root/donatello_ws/install/setup.bash
cd /root/jar_rescate_prueba
bash scripts/probar_rescate.sh 3
```

Terminal 3 dentro del contenedor, largada y evaluación:

```bash
source /opt/ros/humble/setup.bash
source /root/donatello_ws/install/setup.bash
cd /root/jar_rescate_prueba
SIM=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)
python3 tools/juez.py --mundo maze_1_6x5 --mapa "$SIM/maps/maze_1_6x5.yaml" --ground-truth --salida resultado.json
```

`--ground-truth` es exclusivamente del **juez de práctica**, nunca del controlador del equipo. Las colisiones se marcan manualmente: el juez no las detecta solo. No tomar 0 colisiones del JSON como prueba de ausencia de contactos.

Si no se mueve: mirar la razón de freno en la terminal del equipo y verificar `/scan`, cámara, TF, `base_frame` y convergencia de AMCL. No reducir márgenes a ciegas. Si ya existe otro AMCL o publicador map→odom, cerrarlo antes para evitar TF en conflicto. No se recupera automáticamente de una localización fallida ni de un bloqueo dentro del margen de seguridad.

## Validación necesaria antes de aumentar velocidad

```bash
source /opt/ros/humble/setup.bash
cd /root/jar_rescate_prueba
PYTHONPATH=src/equipo_jar python3 -m pytest -q src/equipo_jar/test
bash tools/verificar.sh --template
bash tools/verificar.sh --template --sim maze_1_6x5
```

Probar también otros mundos y desplazamientos iniciales (`spawn_x`, `spawn_y`, `spawn_yaw` en el simulador). Reiniciar el proceso entre corridas, después de reiniciar el juez y el robot: el registro de víctimas vive durante una única corrida. Comprobar que una revisita no publica de nuevo, que cada víctima tiene su reporte, su error ≤0,25 m, el tiempo del último reporte con puntaje <300 s y el regreso físico ≤0,25 m. Medir y marcar cada contacto.

La cobertura usa observaciones panorámicas y prioriza puntos cercanos; **puede exceder 300 s**. Hace falta medir recorridos para optimizarla. No se promete el bono por tener el contador interno en N.

## Parámetros que hay que calibrar

`src/equipo_jar/params/params.yaml`:

- `radio_robot`: radio conservador del volumen barrido del chasis, medido desde `base_link`. No asumir que 0,28 m sirve para cualquier montaje.
- `margen_seguridad`, `desaceleracion`, `timeout_sensor`: distancia adicional, frenado real y latencia máxima.
- `correccion_centro_caja`: 0,10 m inicial. Corrige superficie observada a centro aproximadamente en dirección radial. **No es reconstrucción geométrica de una caja**: depende de tamaño y orientación y puede sesgar coordenadas. Calibrar contra el centro con el juez; no afirmar precisión de 25 cm sin medir. RGB-D o ajuste de caras de caja sería el siguiente refinamiento.
- `separacion_reportes`: radio de asociación de 0,75 m. Está ligado al error esperado y la separación de víctimas; deriva de proximidad, no de un ID real. Con deriva mayor puede haber duplicados; con umbral excesivo puede fusionar víctimas distintas.
- `sigma_localizacion_max`: umbral de covarianza de AMCL, que no prueba por sí solo la exactitud absoluta.
- `velocidad_lineal_max`: 0,12 m/s de inicio; evaluar 0,14 solo tras corridas repetidas sin contacto y con buen frenado.

Los rangos de rojo y ciertos criterios de confirmación todavía están en código; esta es una versión de desarrollo, no un conjunto terminado de parámetros ajustables para el evento.

## Comprobaciones realizadas en el entorno de desarrollo

Se ejecutaron siete pruebas de lógica pura: revisita sin gasto, presupuesto N, datos no finitos/fotogramas repetidos/estimaciones inestables, ancla fija, frenado, geometría de mapa y obstáculos con vencimiento, y diagonales sin atravesar esquinas (algunas pruebas cubren varios casos). Se verificaron sintaxis Python, dependencias permitidas, estructura del template, parámetros e interfaces estáticas.

No disponibles aquí: ROS 2 Humble, compilación colcon, prueba de launch/QoS/AMCL/TF, simulación, puntaje, tiempo real de misión ni ensayo físico. El CI completo no está confirmado.
