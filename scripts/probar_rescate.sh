#!/usr/bin/env bash
# Run from host or within an already configured ROS 2 Humble container.
set -eo pipefail
cd "$(dirname "$0")/.."
N="${1:-3}"
if ! [[ "$N" =~ ^[1-9][0-9]*$ ]]; then
  echo 'Uso: bash scripts/probar_rescate.sh N (cantidad real de víctimas del mundo)'
  exit 2
fi
if ! [[ -f /opt/ros/humble/setup.bash ]]; then
  echo 'Falta ROS 2 Humble. Ejecutá este script dentro de tu entorno ROS, no en Mint directamente.'
  exit 1
fi
source /opt/ros/humble/setup.bash
# Preserve the simulator underlay already sourced in this terminal/container.
if ! ros2 pkg prefix yahboom_rosmaster_gazebo >/dev/null 2>&1; then
  echo 'Falta cargar el simulador: source /root/donatello_ws/install/setup.bash (o tu workspace).'
  exit 1
fi
for pkg in nav2_amcl nav2_lifecycle_manager cv_bridge tf2_ros; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "Falta dependencia ROS: $pkg. Ver instrucciones de instalación en MEJORAS_RESCATE.md."
    exit 1
  fi
done
# A second movement controller can override braking. Stop before starting another.
if ros2 topic info /cmd_vel 2>/dev/null | python3 -c 'import re,sys; sys.exit(not bool(re.search(r"Publisher count: [1-9]",sys.stdin.read())))'; then
  echo 'Ya existe un publicador en /cmd_vel. Cerrá el controlador/teleop anterior y repetí.'
  exit 1
fi
colcon build --base-paths src --packages-select equipo_jar
source install/setup.bash
exec ros2 launch equipo_jar competencia.launch.py use_sim_time:=true victimas:="$N"
