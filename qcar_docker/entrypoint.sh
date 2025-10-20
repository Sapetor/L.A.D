#!/usr/bin/env bash
set -e 
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# ===== Config =====
: "${STATIC_PORT:=7000}"
: "${WVS_PORT:=8080}"
: "${USER_DEFINED_CORS_FLAG:=${CORS_ALLOW_ORIGIN+x}}"
: "${CORS_ALLOW_ORIGIN:=http://localhost:8000}"
: "${ENABLE_ROSAPI:=1}"
: "${ENABLE_JSP:=1}"
: "${ENABLE_WVS:=0}"
: "${ENABLE_TURTLESIM:=0}"
: "${ENABLE_GAZEBO:=0}"
: "${GAZEBO_WORLD:=}"
# Args opcionales para xacro (ej: "use_camera:=true")
: "${XACRO_ARGS:=}"

if [ "${USER_DEFINED_CORS_FLAG}" != "x" ]; then
  CONFIG_PATH="${IP_CONFIG_PATH:-}"
  if [ -n "${CONFIG_PATH}" ] && [ -r "${CONFIG_PATH}" ]; then
    EXPOSED_IP="$(python3 - "${CONFIG_PATH}" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
except (OSError, json.JSONDecodeError):
    sys.exit(0)

ip = (data.get("exposed_ip") or "").strip()
if ip:
    print(ip)
PY
)"
    if [ -n "${EXPOSED_IP}" ]; then
      CORS_ALLOW_ORIGIN="http://${EXPOSED_IP}:3000"
    fi
  fi
fi

# ===== Rutas desde qcar_description instalado =====
QCAR_DESC_PREFIX="$(ros2 pkg prefix qcar_description)"
QCAR_DESC_SHARE="${QCAR_DESC_PREFIX}/share/qcar_description"
URDF_DIR="${QCAR_DESC_SHARE}/urdf"
MAIN_XACRO="${URDF_DIR}/qcar_ros2.urdf.xacro"
TMP_URDF="/tmp/robot_model/robot.urdf"

echo "[entrypoint] qcar_description: ${QCAR_DESC_SHARE}"
echo "[entrypoint] XACRO: ${MAIN_XACRO}"
[ -f "${MAIN_XACRO}" ] || { echo "[ERROR] No existe ${MAIN_XACRO}"; exit 1; }

mkdir -p /tmp/robot_model
xacro "${MAIN_XACRO}" ${XACRO_ARGS} > "${TMP_URDF}"
grep -q "<robot" "${TMP_URDF}" || { echo "[ERROR] URDF inválido"; exit 1; }

# Convert package:// URIs to file:// URIs for Gazebo compatibility
# This is needed because Gazebo Classic doesn't resolve package:// URIs the same way as ROS 2
sed -i "s|package://qcar_description/|file://${QCAR_DESC_SHARE}/|g" "${TMP_URDF}"
echo "[entrypoint] Converted package:// URIs to file:// URIs for Gazebo"

# Copia al package para servir por HTTP
cp -f "${TMP_URDF}" "${URDF_DIR}/robot_runtime.urdf"
echo "[entrypoint] URDF HTTP: http://localhost:${STATIC_PORT}/qcar_description/urdf/robot_runtime.urdf"

# Start Xvfb for headless Gazebo if enabled
if [ "${ENABLE_GAZEBO}" = "1" ]; then
  echo "[entrypoint] Starting Xvfb for headless Gazebo..."
  Xvfb :99 -screen 0 1024x768x24 &
  export DISPLAY=:99

  # Set Gazebo model path so it can find meshes
  export GAZEBO_MODEL_PATH="${QCAR_DESC_SHARE}:${GAZEBO_MODEL_PATH}"
  echo "[entrypoint] GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}"

  # Disable model database to speed up initialization
  export GAZEBO_MODEL_DATABASE_URI=""
  echo "[entrypoint] Disabled Gazebo model database for faster startup"

  sleep 2
fi

# Build launch arguments
LAUNCH_ARGS=(
  "cors_allow_origin:=${CORS_ALLOW_ORIGIN}"
  "static_port:=${STATIC_PORT}"
  "enable_rosapi:=$([ "${ENABLE_ROSAPI}" = "1" ] && echo true || echo false)"
  "enable_jsp:=$([ "${ENABLE_JSP}" = "1" ] && echo true || echo false)"
  "enable_gazebo:=$([ "${ENABLE_GAZEBO}" = "1" ] && echo true || echo false)"
)

# Only add gazebo_world if it has a value
if [ -n "${GAZEBO_WORLD}" ]; then
  LAUNCH_ARGS+=("gazebo_world:=${GAZEBO_WORLD}")
fi

# Add remaining flags
LAUNCH_ARGS+=(
  "enable_wvs:=$([ "${ENABLE_WVS}" = "1" ] && echo true || echo false)"
  "wvs_port:=${WVS_PORT}"
  "enable_turtlesim:=$([ "${ENABLE_TURTLESIM}" = "1" ] && echo true || echo false)"
)

# Lanza todo con flags (Opción 1)
exec ros2 launch qcar_bringup web_viz.launch.py "${LAUNCH_ARGS[@]}"
