# Satellite Count Spatial Visualization (ROS bag -> ParaView)

This project extracts GNSS positions and satellite counts from a ROS 2 bag (MCAP), then colors:

- a trajectory polyline
- a point-cloud map (if present in the bag)

The scalar used for coloring is the mean number of captured satellites near each position, computed with nearest-neighbor interpolation.

## Install

```bash
python3 -m pip install -r requirements.txt
```

## Typical Commands

### 1. Color the original trajectory VTK without moving it

Use this when you want to keep the mapper trajectory geometry exactly as-is and only add a `sat_count_mean` scalar.

```bash
python3 process_satellite_map.py \
  --bag-dir map_2_nanook/all_sensors_2026_07_21-10_02_29 \
  --fix-topic /emlid/top/fix \
  --nmea-topic /emlid/top/nmea_sentence \
  --skip-map \
  --trajectory-vtk-in map_2_nanook/result/trajectory.vtk \
  --trajectory-vtk-out map_2_nanook/result/trajectory_sat.vtk \
  --output-dir map_2_nanook/result
```

This writes:

- `map_2_nanook/result/trajectory_sat.vtk`

### 2. Generate a light trajectory-only file

Use this when your machine cannot handle the map yet.

```bash
python3 process_satellite_map.py \
  --bag-dir map_2_nanook/all_sensors_2026_07_21-10_02_29 \
  --fix-topic /emlid/top/fix \
  --nmea-topic /emlid/top/nmea_sentence \
  --traj-source gnss \
  --skip-map \
  --traj-max-points 1000 \
  --knn-k 8 \
  --output-dir map_2_nanook/result
```

This writes:

- `map_2_nanook/result/trajectory_sat.vtp`

### 3. Generate a light map with voxel downsampling

Use this when the full map is too heavy for CPU, RAM, or GPU.

```bash
python3 process_satellite_map.py \
  --bag-dir map_2_nanook/all_sensors_2026_07_21-10_02_29 \
  --fix-topic /emlid/top/fix \
  --nmea-topic /emlid/top/nmea_sentence \
  --traj-source gnss \
  --traj-max-points 1000 \
  --map-topic /rslidar128/points \
  --map-voxel-size 1.0 \
  --map-max-points 5000 \
  --knn-k 8 \
  --output-dir map_2_nanook/result
```

Notes:
- `--traj-source gnss` uses the GNSS fix track as trajectory.
- `--traj-source pose` uses a pose topic (default `/vn100/pose`) if you prefer mapper/INS trajectory.
- `--skip-map` disables map extraction entirely.
- `--traj-max-points` uniformly downsamples the generated trajectory.
- `--map-voxel-size` keeps one representative point per voxel while reading the map.
- `--map-max-points` applies a final cap after voxel filtering.
- Satellite count is parsed from NMEA:
  - first choice: GGA field (satellites used)
  - fallback: GSV field (satellites in view)

## Output files

Depending on the mode, the script writes ParaView-readable outputs such as:

- `map_2_nanook/result/trajectory_sat.vtp`
- `map_2_nanook/result/trajectory_sat.vtk`
- `map_2_nanook/result/map_sat.vtp`

And CSV intermediates:

- `map_2_nanook/result/gnss_samples.csv`
- `map_2_nanook/result/trajectory_points.csv`
- `map_2_nanook/result/map_points_sampled.csv`

## Open in ParaView

1. Open `trajectory_sat.vtp`, `trajectory_sat.vtk`, and/or `map_sat.vtp`.
2. Click `Apply`.
3. In the `Color By` dropdown, select scalar `sat_count_mean`.
3. Apply a colormap (e.g. Viridis) and rescale to data range.

If you open the legacy VTK trajectory output, ParaView may default to another existing scalar array from the file. Select `sat_count_mean` manually.

## Tips

- If the map is too heavy, increase `--map-voxel-size` to `2.0` or `5.0`.
- If the map is still too heavy, lower `--map-max-points` to `2000` or `1000`.
- If the trajectory is too heavy, lower `--traj-max-points`.
- If your preferred antenna is not `/emlid/top/*`, change `--fix-topic` and `--nmea-topic`.
- If NMEA does not contain GGA/GSV, satellite count cannot be extracted and script will fail with guidance.
