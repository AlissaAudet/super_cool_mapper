#!/bin/bash
set -eo pipefail

# ROS2 setup files use unbound variables internally — disable -u around them
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

BAG_PATH="${BAG_PATH:-/bags}"
TOPIC="${POINT_CLOUD_TOPIC:-/lidar/point_cloud}"
# Always mounted from the standalone folder's config/mapper.yaml by run-mapper.sh
MAP_CONFIG="/config/mapper.yaml"
MAP_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
# MAP_NAME can be passed via docker -e MAP_NAME=workshop_map from the app
if [ -n "${MAP_NAME:-}" ]; then
    SAFE_NAME=$(echo "$MAP_NAME" | tr -cd 'a-zA-Z0-9_-')
    FINAL_MAP="/maps/${SAFE_NAME}.vtk"
else
    FINAL_MAP="/maps/map_${MAP_TIMESTAMP}.vtk"
fi

echo "[mapper] === Norlab ICP Mapper for ANYmal ==="
echo "[mapper] Bag:    $BAG_PATH"
echo "[mapper] Topic:  $TOPIC  (remapped ← points_in)"
echo "[mapper] Config: $MAP_CONFIG"
echo "[mapper] Output: $FINAL_MAP"
echo ""

# Clean shutdown on SIGTERM / SIGINT
_term() {
    echo "[mapper] Shutdown signal received."
    echo "[mapper] Saving map to: $FINAL_MAP ..."

    # 1. Call the ROS2 save service explicitly (most reliable method)
    # Service is advertised as "save_map" with no namespace (see
    # norlab_icp_mapper_ros/src/mapper_node.cpp), so it resolves to /save_map —
    # not /mapper_node/save_map. `ros2 service call` has no built-in timeout
    # flag, so wrap it in `timeout` to avoid hanging forever if unreachable.
    timeout 20 ros2 service call /save_map \
        norlab_icp_mapper_ros/srv/SaveMap \
        "{map_file_name: {data: '${FINAL_MAP}'}}" 2>/dev/null \
    && echo "[mapper] Map saved via service call." \
    || echo "[mapper] Service call timed out — relying on final_map_file_name."

    # 2. Gracefully stop the mapper (will also trigger final_map_file_name save)
    echo "[mapper] Stopping mapper node..."
    kill "$MAPPER_PID" 2>/dev/null || true
    wait "$MAPPER_PID" 2>/dev/null || true
    echo "[mapper] Done. File: $FINAL_MAP"
    exit 0
}
trap _term TERM INT

# ── Static TF for the ANYmal lidar chain ──────────────────────────────────────
# Values extracted from the bag: base→lidar_parent→lidar
# Published with use_sim_time=true so they align with bag timestamps.
ros2 run tf2_ros static_transform_publisher \
    --x -0.31 --y 0.0 --z 0.1585 \
    --qx 0.0 --qy 0.0 --qz 0.7071 --qw 0.7071 \
    --frame-id base --child-frame-id lidar_parent \
    --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --qx 0.0 --qy 0.0 --qz 0.7071 --qw 0.7071 \
    --frame-id lidar_parent --child-frame-id lidar \
    --ros-args -p use_sim_time:=true &
echo "[mapper] base→lidar_parent→lidar static TF publishers started."

# ── odom→base TF bridge (converts odometry to TF at bag timestamps) ──────────
# /legged_odometry/pose_in_odom is PoseWithCovarianceStamped giving pose of
# 'base' in 'odom'. We republish as TF using the message's own stamp so the
# TF timestamps match the bag's point cloud timestamps (both in sim time).
python3 - << 'PYEOF' &
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge',
            parameter_overrides=[Parameter('use_sim_time',
                Parameter.Type.BOOL, True)])
        self.br = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/legged_odometry/pose_in_odom', self.cb, 100)
        self.get_logger().info('odom→base TF bridge ready (sim_time=true).')

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # bag timestamp → aligns with point clouds
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

from rclpy.executors import ExternalShutdownException
rclpy.init()
node = OdomToTF()
try:
    rclpy.spin(node)
except (ExternalShutdownException, KeyboardInterrupt):
    pass
PYEOF
ODOM_TF_PID=$!
echo "[mapper] odom→base TF bridge started (PID $ODOM_TF_PID)."
sleep 2

# ── Mapper node ───────────────────────────────────────────────────────────────
# use_sim_time=true: all timestamps use the bag's /clock so TF lookups align.
# robot_frame=base: ANYmal root link. TF chain: odom→base→lidar_parent→lidar.
echo "[mapper] Starting mapper node..."
ros2 run norlab_icp_mapper_ros mapper_node \
    --ros-args \
    -p "mapping_config:=$MAP_CONFIG" \
    -p "use_sim_time:=true" \
    -p "is_online:=true" \
    -p "is_mapping:=true" \
    -p "is_3D:=true" \
    -p "save_map_cells_on_hard_drive:=false" \
    -p "robot_frame:=base" \
    -p "odom_frame:=odom" \
    -p "final_map_file_name:=$FINAL_MAP" \
    -r "points_in:=$TOPIC" \
    >/tmp/mapper_node.log 2>&1 &
MAPPER_PID=$!

echo "[mapper] Waiting for mapper to initialize (15s)..."
sleep 15

if ! kill -0 "$MAPPER_PID" 2>/dev/null; then
    echo "[mapper] ERROR: Mapper node crashed during startup. Last log lines:"
    tail -20 /tmp/mapper_node.log 2>/dev/null || true
    exit 1
fi

# ── Bag playback ──────────────────────────────────────────────────────────────
# --clock: publishes /clock so all use_sim_time=true nodes get bag timestamps.
# Only the 3 topics the mapper needs — avoids the ANYmal SLAM TF loop.
echo "[mapper] Mapper ready. Starting bag playback..."

# Progress reporter — prints every 15s so the app panel stays alive
(
  START_T=$(date +%s)
  INTERVAL=15
  while true; do
    sleep $INTERVAL
    ELAPSED=$(( $(date +%s) - START_T ))
    ERRORS=$(grep -c 'callback failed' /tmp/mapper_node.log 2>/dev/null || echo 0)
    echo "[mapper] Mapping in progress... ${ELAPSED}s elapsed | TF errors: ${ERRORS}"
  done
) &
PROGRESS_PID=$!

ros2 bag play "$BAG_PATH" --clock \
    --topics /lidar/point_cloud /legged_odometry/pose_in_odom /tf_static || {
    echo "[mapper] Bag playback ended (exit code: $?)"
}
kill $PROGRESS_PID 2>/dev/null || true
kill $ODOM_TF_PID 2>/dev/null || true

echo ""
echo "[mapper] ================================================"
echo "[mapper] Bag playback complete!"
echo "[mapper] Mapper is still running and building the map."
echo "[mapper] The map will be AUTO-SAVED to:"
echo "[mapper]   $FINAL_MAP"
echo "[mapper] when the container stops (click Stop in app)."
echo "[mapper] Or use Save Map button for immediate save."
echo "[mapper] ================================================"

wait "$MAPPER_PID"
