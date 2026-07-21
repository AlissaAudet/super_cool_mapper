#!/usr/bin/env bash
# Convertit (si besoin) un bag ROS1 en ROS2 MCAP, puis lance le mapper NorLab ICP
# dans Docker. Ctrl+C une fois la lecture du bag terminée pour sauvegarder la carte.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="anymal-norlab-mapper"

usage() {
  cat <<EOF
Usage: $0 <bag_path> <output_dir> [map_name] [point_cloud_topic]

  bag_path            Fichier .bag ROS1, OU dossier ROS2 MCAP déjà converti
  output_dir          Dossier où écrire la carte .vtk résultante
  map_name            Optionnel. Nom de base du fichier (def: map_<timestamp>.vtk)
  point_cloud_topic   Optionnel. Topic LiDAR (def: /lidar/point_cloud)

Exemple:
  $0 ~/bags/session1.bag ~/maps mission1
EOF
  exit 1
}

[ $# -lt 2 ] && usage

BAG_INPUT="$1"
OUTPUT_DIR="$2"
MAP_NAME="${3:-}"
TOPIC="${4:-/lidar/point_cloud}"

[ -e "$BAG_INPUT" ] || { echo "Introuvable: $BAG_INPUT"; exit 1; }
mkdir -p "$OUTPUT_DIR"

MAPPER_CONFIG="$SCRIPT_DIR/config/mapper.yaml"
[ -f "$MAPPER_CONFIG" ] || { echo "Introuvable: $MAPPER_CONFIG"; exit 1; }

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "Image Docker '$IMAGE_NAME' introuvable. Lancer ./setup.sh d'abord."
  exit 1
fi

CLEANUP_DIR=""
cleanup() { [ -n "$CLEANUP_DIR" ] && rm -rf "$CLEANUP_DIR"; }
trap cleanup EXIT

EFFECTIVE_BAG="$BAG_INPUT"
if [[ "$BAG_INPUT" == *.bag ]]; then
  echo "=== Conversion du bag ROS1 vers ROS2 MCAP ==="
  CLEANUP_DIR="$(mktemp -d)"
  MCAP_DIR="$CLEANUP_DIR/$(basename "${BAG_INPUT%.bag}")"
  PYTHON="$SCRIPT_DIR/.venv/bin/python3"
  [ -x "$PYTHON" ] || PYTHON="python3"
  "$PYTHON" "$SCRIPT_DIR/scripts/convert_bag.py" "$BAG_INPUT" "$MCAP_DIR" ros2
  EFFECTIVE_BAG="$MCAP_DIR"
  echo ""
fi

echo "=== Démarrage du conteneur mapper ==="
ENV_ARGS=(-e "POINT_CLOUD_TOPIC=$TOPIC")
[ -n "$MAP_NAME" ] && ENV_ARGS+=(-e "MAP_NAME=$MAP_NAME")

VOLUME_ARGS=(
  -v "$(realpath "$EFFECTIVE_BAG"):/bags:ro"
  -v "$(realpath "$OUTPUT_DIR"):/maps"
  -v "$MAPPER_CONFIG:/config/mapper.yaml:ro"
)
echo "Config ICP: $MAPPER_CONFIG"

CID=$(docker run -d \
  "${VOLUME_ARGS[@]}" \
  "${ENV_ARGS[@]}" \
  "$IMAGE_NAME")

echo "Conteneur: $CID"
echo "Logs en direct ci-dessous. Une fois 'Bag playback complete' affiché,"
echo "appuyer sur Ctrl+C pour arrêter proprement et sauvegarder la carte."
echo ""

stop_container() {
  echo ""
  echo "=== Arrêt du conteneur (déclenche la sauvegarde, jusqu'à 90s) ==="
  docker stop --time 90 "$CID" >/dev/null
  echo "Carte sauvegardée dans: $OUTPUT_DIR"
  exit 0
}
trap stop_container INT TERM

docker logs -f "$CID"
