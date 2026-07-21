#!/usr/bin/env bash
# One-time setup: Python venv (bag conversion) + Docker image (norlab_icp_mapper stack).
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="anymal-norlab-mapper"

echo "=== 1/2: Python venv pour la conversion de bags (rosbags) ==="
if ! command -v python3 &>/dev/null; then
  echo "python3 introuvable. Installer avec: sudo apt install python3 python3-venv"
  exit 1
fi
if [ ! -d "$SCRIPT_DIR/.venv" ]; then
  python3 -m venv "$SCRIPT_DIR/.venv" 2>/dev/null || {
    echo "python3-venv manquant — installation (sudo requis)..."
    sudo apt-get install -y python3-venv python3-pip
    python3 -m venv "$SCRIPT_DIR/.venv"
  }
fi
"$SCRIPT_DIR/.venv/bin/pip" install --upgrade pip --quiet
echo "Installation de rosbags (avec support MCAP) et pyyaml..."
# pyyaml n'est plus une dépendance transitive de rosbags (voir versions récentes) —
# convert_bag.py en a pourtant besoin directement pour patcher metadata.yaml.
"$SCRIPT_DIR/.venv/bin/pip" install "rosbags[mcap]" pyyaml --quiet
echo "venv prêt."
echo ""

echo "=== 2/2: Image Docker (stack NorLab ICP Mapper) ==="
if ! command -v docker &>/dev/null; then
  echo "Docker introuvable. Installer avec: sudo apt install docker.io"
  echo "Puis: sudo usermod -aG docker \$USER   (et se déconnecter/reconnecter)"
  exit 1
fi

if docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "L'image '$IMAGE_NAME' existe déjà. Rien à faire."
  echo "(Pour forcer une reconstruction: docker rmi $IMAGE_NAME, puis relancer ce script.)"
else
  echo "Construction de l'image — compile ROS2 Humble + la stack NorLab ICP au complet."
  echo "Ça prend 30 à 60 minutes et nécessite une connexion internet. Ne pas interrompre."
  read -p "Continuer maintenant ? [y/N] " ans
  if [[ "$ans" =~ ^[Yy]$ ]]; then
    docker build -f "$SCRIPT_DIR/docker/Dockerfile.norlab-mapper" -t "$IMAGE_NAME" "$SCRIPT_DIR"
    echo "Image '$IMAGE_NAME' construite avec succès."
  else
    echo "Construction ignorée. Relancer ce script plus tard, ou manuellement:"
    echo "  docker build -f docker/Dockerfile.norlab-mapper -t $IMAGE_NAME ."
  fi
fi
