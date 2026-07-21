# NorLab Mapper — standalone

Version autonome (sans l'app web ANYmal Toolkit) pour générer une carte 3D
(`.vtk`) à partir d'un bag ROS1 de l'ANYmal, avec le mapper NorLab ICP tournant
dans Docker.

Ce dossier est un extrait de `ANYmal-help-app` (copié, pas déplacé — l'app
originale reste intacte). Il est pensé pour être copié tel quel sur un autre
ordinateur.

## Contenu

```
norlab-mapper-standalone/
├── setup.sh                        Installation unique (venv Python + image Docker)
├── run-mapper.sh                   Lance la conversion + le mapper sur un bag
├── docker/
│   ├── Dockerfile.norlab-mapper    Image: ROS2 Humble + stack NorLab ICP Mapper
│   └── entrypoint-mapper.sh        Script exécuté dans le conteneur (TF, lecture du bag, sauvegarde)
├── config/
│   └── mapper.yaml                 Config ICP du mapper (filtres, matcher, etc.) — éditable sans rebuild
└── scripts/
    └── convert_bag.py              Conversion ROS1 .bag → ROS2 MCAP (requis avant le mapping)
```

## Prérequis

- Linux (Ubuntu 22.04/24.04)
- Docker (`sudo apt install docker.io`, puis `sudo usermod -aG docker $USER` et se reconnecter)
- Python 3 (`sudo apt install python3 python3-venv`)
- Une connexion internet (uniquement pour la construction de l'image, une seule fois)

## Installation (une seule fois par ordinateur)

```bash
cd norlab-mapper-standalone
./setup.sh
```

Ce script fait deux choses :

1. Crée un venv Python local (`.venv/`) avec `rosbags[mcap]`, nécessaire pour
   convertir les bags ROS1 en ROS2 MCAP.
2. Construit l'image Docker `anymal-norlab-mapper`. **Cette étape compile ROS2
   et toute la stack NorLab (libnabo, libpointmatcher, norlab_icp_mapper) à
   partir des sources — ça prend 30 à 60 minutes.** À ne faire qu'une seule
   fois par machine (l'image reste ensuite en cache Docker).

Si vous déployez sur un autre ordinateur : copiez tout le dossier
`norlab-mapper-standalone/` et relancez `./setup.sh` sur cette machine — le
venv et l'image Docker sont propres à chaque machine et ne se copient pas.

## Utilisation

```bash
./run-mapper.sh <bag> <dossier_sortie> [nom_carte] [topic_lidar]
```

- `<bag>` : chemin vers un fichier `.bag` ROS1, **ou** un dossier ROS2 MCAP déjà converti.
- `<dossier_sortie>` : où écrire le fichier `.vtk` final (créé si absent).
- `[nom_carte]` : optionnel, nom de base du fichier (ex: `mission1` → `mission1.vtk`). Par défaut : `map_<timestamp>.vtk`.
- `[topic_lidar]` : optionnel, topic du nuage de points. Par défaut : `/lidar/point_cloud` (topic standard ANYmal).

Exemple :

```bash
./run-mapper.sh ~/bags/session1.bag ~/maps mission1
```

Ce qui se passe :

1. Si le bag est un `.bag` ROS1, il est d'abord converti en ROS2 MCAP dans un
   dossier temporaire (auto-supprimé à la fin).
2. Le conteneur Docker démarre : il publie les transformations statiques
   base→lidar de l'ANYmal, convertit l'odométrie en TF, démarre le nœud
   mapper, puis rejoue le bag.
3. Les logs s'affichent en direct dans le terminal.
4. Quand `[mapper] Bag playback complete!` apparaît, le bag est terminé mais
   **le mapper tourne toujours** (il continue de fusionner les scans).
   **Appuyez sur Ctrl+C** pour arrêter proprement le conteneur — ça déclenche
   la sauvegarde finale de la carte (jusqu'à 90 secondes).
5. Le fichier `.vtk` apparaît dans `<dossier_sortie>`.

Le résultat peut ensuite être ouvert avec ParaView :

```bash
sudo apt install paraview
paraview ~/maps/mission1.vtk
```

## Config ICP (`config/mapper.yaml`)

`run-mapper.sh` monte **toujours** `config/mapper.yaml` dans le conteneur, à
l'emplacement fixe `/config/mapper.yaml` que `entrypoint-mapper.sh` utilise
directement (pas de valeur par défaut alternative). Ça veut dire que **les
paramètres ICP (filtres, matcher, seuils, etc.) se modifient directement dans
ce fichier, sans reconstruire l'image Docker** — un nouveau
`./run-mapper.sh` suffit pour que le changement prenne effet. Si le fichier
est absent, `run-mapper.sh` refuse de démarrer (message clair au lieu d'un
échec silencieux dans le conteneur).

Le fichier fourni ici est la config par défaut du package
`norlab_icp_mapper_ros` (extraite de l'image `anymal-norlab-mapper`), **non
retouchée pour l'ANYmal** — c'est un point de départ, pas une config
optimisée pour le VLP-16.

## Notes

- Les topics attendus dans le bag sont ceux standards de l'ANYmal :
  `/lidar/point_cloud` (LiDAR) et `/legged_odometry/pose_in_odom` (odométrie).
  Si votre bag utilise d'autres noms de topics, il faut éditer la liste
  `--topics` dans `docker/entrypoint-mapper.sh` (recherchez `ros2 bag play`).
- Contrairement à l'app web, `run-mapper.sh` n'arrête pas automatiquement une
  instance déjà en cours. Vérifiez qu'aucun conteneur `anymal-norlab-mapper`
  ne tourne déjà avant de relancer : `docker ps --filter ancestor=anymal-norlab-mapper`.
- Pour reconstruire l'image après une mise à jour du Dockerfile :
  `docker rmi anymal-norlab-mapper && ./setup.sh`.
