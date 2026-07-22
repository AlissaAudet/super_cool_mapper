#!/usr/bin/env python3
"""Build satellite-count-colored trajectory/map outputs from a ROS 2 bag.

Pipeline:
1. Read GNSS fixes and NMEA sentences from the bag.
2. Parse satellite counts from NMEA (GGA preferred, GSV fallback).
3. Attach a satellite count to each GNSS position by nearest timestamp.
4. Build trajectory points (GNSS or pose topic).
5. Read map points from PointCloud2 topic (sampled).
6. Interpolate mean satellite counts with k-NN (inverse-distance weighting).
7. Export ParaView-ready VTK PolyData (.vtp) and CSV intermediates.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import yaml
from rosbags.highlevel import AnyReader


EARTH_RADIUS_M = 6378137.0


@dataclass
class TimedPosition:
    t_ns: int
    x: float
    y: float
    z: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create satellite-colored map/trajectory files for ParaView.")
    parser.add_argument("--bag-dir", required=True, help="Path to ROS 2 bag directory (contains metadata.yaml + mcap/db3).")
    parser.add_argument("--fix-topic", default="/emlid/top/fix", help="NavSatFix topic for antenna positions.")
    parser.add_argument("--nmea-topic", default="/emlid/top/nmea_sentence", help="NMEA sentence topic for satellite count parsing.")
    parser.add_argument("--traj-source", choices=["gnss", "pose"], default="gnss", help="Trajectory source: GNSS fixes or pose topic.")
    parser.add_argument("--pose-topic", default="/vn100/pose", help="Pose topic used when --traj-source pose.")
    parser.add_argument("--map-topic", default="/rslidar128/points", help="PointCloud2 topic used as map points.")
    parser.add_argument("--skip-map", action="store_true", help="Skip map extraction/interpolation and only export trajectory.")
    parser.add_argument("--max-time-diff-s", type=float, default=1.0, help="Max timestamp diff between fix and NMEA for matching.")
    parser.add_argument("--knn-k", type=int, default=8, help="Number of neighbors for interpolation.")
    parser.add_argument(
        "--map-max-points",
        type=int,
        default=500000,
        help="Maximum map points to keep (random sampling) to limit memory/output size.",
    )
    parser.add_argument(
        "--map-voxel-size",
        type=float,
        default=0.0,
        help="If > 0, downsample map points with a voxel grid of this size before interpolation/export.",
    )
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducible map sampling.")
    parser.add_argument(
        "--traj-max-points",
        type=int,
        default=0,
        help="If > 0, downsample trajectory to at most this many points (uniform index sampling).",
    )
    parser.add_argument(
        "--trajectory-vtk-in",
        default="",
        help="Optional input legacy ASCII VTK trajectory file to preserve geometry exactly and only add color scalar.",
    )
    parser.add_argument(
        "--trajectory-vtk-out",
        default="",
        help="Output legacy VTK file path for colored trajectory (used with --trajectory-vtk-in).",
    )
    parser.add_argument("--output-dir", default="outputs", help="Output directory for VTK and CSV files.")
    return parser.parse_args()


def get_bag_data_paths(bag_dir: Path) -> List[Path]:
    metadata = bag_dir / "metadata.yaml"
    if not metadata.exists():
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")

    with metadata.open("r", encoding="utf-8") as fh:
        md = yaml.safe_load(fh)

    rel_paths = md.get("rosbag2_bagfile_information", {}).get("relative_file_paths", [])
    paths = [bag_dir / rel for rel in rel_paths]
    missing = [p for p in paths if not p.exists()]
    if missing:
        raise FileNotFoundError(f"Bag data files missing: {missing}")
    return paths


def ns_from_stamp(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nmea_payload(sentence: str) -> Tuple[str, List[str]]:
    line = sentence.strip()
    if not line or "$" not in line:
        return "", []

    start = line.find("$") + 1
    core = line[start:]
    if "*" in core:
        core = core.split("*", 1)[0]

    fields = core.split(",")
    if not fields:
        return "", []

    talker_type = fields[0]
    msg_type = talker_type[-3:] if len(talker_type) >= 3 else talker_type
    return msg_type.upper(), fields


def sat_count_from_nmea(sentence: str) -> Optional[int]:
    msg_type, fields = nmea_payload(sentence)

    if msg_type == "GGA":
        if len(fields) > 7 and fields[7]:
            try:
                val = int(fields[7])
                if val >= 0:
                    return val
            except ValueError:
                return None

    if msg_type == "GSV":
        if len(fields) > 3 and fields[3]:
            try:
                val = int(fields[3])
                if val >= 0:
                    return val
            except ValueError:
                return None

    return None


def parse_pointcloud2_xyz(msg) -> np.ndarray:
    # PointCloud2 binary decoding using x,y,z field offsets.
    field_by_name = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"}.issubset(field_by_name):
        return np.empty((0, 3), dtype=np.float64)

    x_off = int(field_by_name["x"].offset)
    y_off = int(field_by_name["y"].offset)
    z_off = int(field_by_name["z"].offset)
    step = int(msg.point_step)

    if step <= 0:
        return np.empty((0, 3), dtype=np.float64)

    data = msg.data
    num_points = len(data) // step
    out = np.empty((num_points, 3), dtype=np.float64)

    for i in range(num_points):
        base = i * step
        out[i, 0] = struct.unpack_from("<f", data, base + x_off)[0]
        out[i, 1] = struct.unpack_from("<f", data, base + y_off)[0]
        out[i, 2] = struct.unpack_from("<f", data, base + z_off)[0]

    finite = np.isfinite(out).all(axis=1)
    return out[finite]


def geodetic_to_local_xy(lat_deg: np.ndarray, lon_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    lat0 = math.radians(float(lat_deg[0]))
    lon0 = math.radians(float(lon_deg[0]))

    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)

    x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
    y = (lat - lat0) * EARTH_RADIUS_M
    return x, y


def nearest_value_by_time(
    query_ns: np.ndarray,
    ref_ns: np.ndarray,
    ref_values: np.ndarray,
    max_diff_ns: int,
) -> np.ndarray:
    idx = np.searchsorted(ref_ns, query_ns, side="left")

    left_idx = np.clip(idx - 1, 0, len(ref_ns) - 1)
    right_idx = np.clip(idx, 0, len(ref_ns) - 1)

    left_diff = np.abs(query_ns - ref_ns[left_idx])
    right_diff = np.abs(query_ns - ref_ns[right_idx])

    use_right = right_diff < left_diff
    best_idx = np.where(use_right, right_idx, left_idx)
    best_diff = np.where(use_right, right_diff, left_diff)

    out = ref_values[best_idx].astype(np.float64)
    out[best_diff > max_diff_ns] = np.nan
    return out


def interpolate_knn(
    src_xyz: np.ndarray,
    src_val: np.ndarray,
    dst_xyz: np.ndarray,
    k: int,
) -> np.ndarray:
    if len(src_xyz) == 0:
        raise ValueError("No source samples available for interpolation.")
    if len(dst_xyz) == 0:
        return np.empty((0,), dtype=np.float64)

    from sklearn.neighbors import NearestNeighbors

    k_eff = max(1, min(k, len(src_xyz)))
    nn = NearestNeighbors(n_neighbors=k_eff)
    nn.fit(src_xyz)
    dist, ind = nn.kneighbors(dst_xyz)

    # Inverse distance weighting with epsilon for exact matches.
    w = 1.0 / np.maximum(dist, 1e-6)
    vals = src_val[ind]
    return np.sum(w * vals, axis=1) / np.sum(w, axis=1)


def uniform_sample_indices(n: int, max_points: int) -> np.ndarray:
    if max_points <= 0 or n <= max_points:
        return np.arange(n, dtype=np.int64)
    return np.linspace(0, n - 1, num=max_points, dtype=np.int64)


def voxel_keys(points_xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    return np.floor(points_xyz / voxel_size).astype(np.int64)


def update_voxel_map(
    voxel_map: Dict[Tuple[int, int, int], Tuple[float, float, float]],
    points_xyz: np.ndarray,
    voxel_size: float,
) -> None:
    if len(points_xyz) == 0:
        return

    keys = voxel_keys(points_xyz, voxel_size)
    for point, key in zip(points_xyz, keys):
        voxel_tuple = (int(key[0]), int(key[1]), int(key[2]))
        if voxel_tuple not in voxel_map:
            voxel_map[voxel_tuple] = (float(point[0]), float(point[1]), float(point[2]))


def resample_series_by_index(values: np.ndarray, target_n: int) -> np.ndarray:
    if target_n <= 0:
        return np.empty((0,), dtype=np.float64)
    if len(values) == 0:
        raise ValueError("Cannot resample from an empty value series.")
    if len(values) == target_n:
        return values.astype(np.float64)
    if len(values) == 1:
        return np.full((target_n,), float(values[0]), dtype=np.float64)

    src_x = np.linspace(0.0, 1.0, num=len(values), dtype=np.float64)
    dst_x = np.linspace(0.0, 1.0, num=target_n, dtype=np.float64)
    return np.interp(dst_x, src_x, values).astype(np.float64)


def read_legacy_vtk_points_count(path: Path) -> int:
    text = path.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"^\s*POINTS\s+(\d+)\s+", text, flags=re.MULTILINE)
    if not m:
        raise ValueError(f"Could not parse POINTS count from VTK file: {path}")
    return int(m.group(1))


def write_colored_legacy_vtk_copy(input_path: Path, output_path: Path, sat_values: np.ndarray) -> None:
    text = input_path.read_text(encoding="utf-8", errors="ignore")
    n_points = read_legacy_vtk_points_count(input_path)
    if len(sat_values) != n_points:
        raise ValueError(
            f"Scalar length mismatch for VTK export: got {len(sat_values)} values for {n_points} points."
        )

    scalar_lines = [
        "SCALARS sat_count_mean float 1",
        "LOOKUP_TABLE default",
    ]
    scalar_lines.extend(f"{float(v):.6f}" for v in sat_values)
    scalar_block = "\n".join(scalar_lines) + "\n"

    point_data_match = re.search(r"^\s*POINT_DATA\s+(\d+)\s*$", text, flags=re.MULTILINE)
    if point_data_match:
        pd_count = int(point_data_match.group(1))
        if pd_count != n_points:
            raise ValueError(
                f"POINT_DATA count mismatch in input VTK: POINT_DATA={pd_count}, POINTS={n_points}."
            )

        cell_data_match = re.search(r"^\s*CELL_DATA\s+\d+\s*$", text[point_data_match.end():], flags=re.MULTILINE)
        if cell_data_match:
            insert_at = point_data_match.end() + cell_data_match.start()
            new_text = text[:insert_at].rstrip() + "\n" + scalar_block + text[insert_at:]
        else:
            new_text = text.rstrip() + "\n" + scalar_block
    else:
        new_text = text.rstrip() + "\n" + f"POINT_DATA {n_points}\n" + scalar_block

    output_path.write_text(new_text, encoding="utf-8")


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence[float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


def vtk_data_array(name: str, num_comp: int, values: Iterable[float], dtype: str = "Float64") -> str:
    vals = " ".join(f"{float(v):.6f}" for v in values)
    return (
        f'<DataArray type="{dtype}" Name="{name}" NumberOfComponents="{num_comp}" format="ascii">\n'
        f"{vals}\n"
        f"</DataArray>\n"
    )


def write_vtp_points(path: Path, points_xyz: np.ndarray, scalars: Dict[str, np.ndarray]) -> None:
    n = len(points_xyz)
    pts_flat = points_xyz.reshape(-1)

    point_data = ""
    for name, arr in scalars.items():
        point_data += vtk_data_array(name, 1, arr)

    xml = []
    xml.append('<?xml version="1.0"?>\n')
    xml.append('<VTKFile type="PolyData" version="0.1" byte_order="LittleEndian">\n')
    xml.append("<PolyData>\n")
    xml.append(f'<Piece NumberOfPoints="{n}" NumberOfVerts="0" NumberOfLines="0" NumberOfStrips="0" NumberOfPolys="0">\n')
    xml.append("<PointData Scalars=\"sat_count_mean\">\n")
    xml.append(point_data)
    xml.append("</PointData>\n")
    xml.append("<Points>\n")
    xml.append(vtk_data_array("Points", 3, pts_flat))
    xml.append("</Points>\n")
    xml.append("</Piece>\n")
    xml.append("</PolyData>\n")
    xml.append("</VTKFile>\n")

    path.write_text("".join(xml), encoding="utf-8")


def write_vtp_polyline(path: Path, points_xyz: np.ndarray, scalars: Dict[str, np.ndarray]) -> None:
    n = len(points_xyz)
    pts_flat = points_xyz.reshape(-1)

    point_data = ""
    for name, arr in scalars.items():
        point_data += vtk_data_array(name, 1, arr)

    connectivity = " ".join(str(i) for i in range(n))
    offsets = str(n)

    xml = []
    xml.append('<?xml version="1.0"?>\n')
    xml.append('<VTKFile type="PolyData" version="0.1" byte_order="LittleEndian">\n')
    xml.append("<PolyData>\n")
    xml.append(f'<Piece NumberOfPoints="{n}" NumberOfVerts="0" NumberOfLines="1" NumberOfStrips="0" NumberOfPolys="0">\n')
    xml.append("<PointData Scalars=\"sat_count_mean\">\n")
    xml.append(point_data)
    xml.append("</PointData>\n")
    xml.append("<Points>\n")
    xml.append(vtk_data_array("Points", 3, pts_flat))
    xml.append("</Points>\n")
    xml.append("<Lines>\n")
    xml.append(
        '<DataArray type="Int32" Name="connectivity" format="ascii">\n'
        f"{connectivity}\n"
        "</DataArray>\n"
    )
    xml.append(
        '<DataArray type="Int32" Name="offsets" format="ascii">\n'
        f"{offsets}\n"
        "</DataArray>\n"
    )
    xml.append("</Lines>\n")
    xml.append("</Piece>\n")
    xml.append("</PolyData>\n")
    xml.append("</VTKFile>\n")

    path.write_text("".join(xml), encoding="utf-8")


def main() -> None:
    args = parse_args()
    random.seed(args.seed)

    bag_dir = Path(args.bag_dir).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    bag_paths = get_bag_data_paths(bag_dir)

    fixes: List[Tuple[int, float, float, float]] = []
    nmea_counts: List[Tuple[int, int]] = []
    poses: List[TimedPosition] = []
    map_points: List[Tuple[float, float, float]] = []
    map_voxels: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}

    with AnyReader(bag_paths) as reader:
        conns = {c.topic: c for c in reader.connections}

        required = [args.fix_topic, args.nmea_topic]
        missing = [t for t in required if t not in conns]
        if missing:
            available = sorted(conns.keys())
            raise ValueError(f"Missing required topics: {missing}. Available: {available}")

        selected_topics = {args.fix_topic, args.nmea_topic, args.pose_topic}
        if not args.skip_map:
            selected_topics.add(args.map_topic)

        for conn, t_ns, rawdata in reader.messages(connections=[c for c in reader.connections if c.topic in selected_topics]):
            msg = reader.deserialize(rawdata, conn.msgtype)

            if conn.topic == args.fix_topic:
                fixes.append((t_ns, float(msg.latitude), float(msg.longitude), float(msg.altitude)))

            elif conn.topic == args.nmea_topic:
                count = sat_count_from_nmea(str(msg.sentence))
                if count is not None:
                    nmea_counts.append((t_ns, int(count)))

            elif conn.topic == args.pose_topic and args.traj_source == "pose":
                p = msg.pose.pose.position
                poses.append(TimedPosition(t_ns=t_ns, x=float(p.x), y=float(p.y), z=float(p.z)))

            elif not args.skip_map and conn.topic == args.map_topic:
                pts = parse_pointcloud2_xyz(msg)
                if len(pts) > 0:
                    if args.map_voxel_size > 0.0:
                        update_voxel_map(map_voxels, pts, args.map_voxel_size)
                    else:
                        map_points.extend(map(tuple, pts.tolist()))

    if not fixes:
        raise ValueError(f"No fixes read from topic {args.fix_topic}")

    if not nmea_counts:
        raise ValueError(
            f"No satellite count could be parsed from topic {args.nmea_topic}. "
            "Expected GGA and/or GSV NMEA sentences."
        )

    fix_df = pd.DataFrame(fixes, columns=["t_ns", "lat", "lon", "alt"])
    nmea_df = pd.DataFrame(nmea_counts, columns=["t_ns", "sat_count"]).sort_values("t_ns").reset_index(drop=True)

    x, y = geodetic_to_local_xy(fix_df["lat"].to_numpy(), fix_df["lon"].to_numpy())
    fix_df["x"] = x
    fix_df["y"] = y
    fix_df["z"] = fix_df["alt"]

    sat = nearest_value_by_time(
        query_ns=fix_df["t_ns"].to_numpy(dtype=np.int64),
        ref_ns=nmea_df["t_ns"].to_numpy(dtype=np.int64),
        ref_values=nmea_df["sat_count"].to_numpy(dtype=np.int64),
        max_diff_ns=int(args.max_time_diff_s * 1e9),
    )
    fix_df["sat_count"] = sat

    valid_fix_df = fix_df.dropna(subset=["sat_count"]).copy()
    if valid_fix_df.empty:
        raise ValueError("All fix samples failed time matching with NMEA satellite counts. Increase --max-time-diff-s.")

    valid_fix_df["sat_count"] = valid_fix_df["sat_count"].astype(float)

    src_xyz = valid_fix_df[["x", "y", "z"]].to_numpy(dtype=np.float64)
    src_sat = valid_fix_df["sat_count"].to_numpy(dtype=np.float64)

    # Trajectory selection.
    trajectory_vtk_in = Path(args.trajectory_vtk_in).resolve() if args.trajectory_vtk_in else None
    trajectory_vtk_out = Path(args.trajectory_vtk_out).resolve() if args.trajectory_vtk_out else None

    if trajectory_vtk_in is not None:
        n_traj_points = read_legacy_vtk_points_count(trajectory_vtk_in)
        traj_sat = resample_series_by_index(src_sat, n_traj_points)
        traj_df = pd.DataFrame({
            "t_ns": np.full((n_traj_points,), np.nan),
            "x": np.full((n_traj_points,), np.nan),
            "y": np.full((n_traj_points,), np.nan),
            "z": np.full((n_traj_points,), np.nan),
        })
        traj_xyz = np.empty((0, 3), dtype=np.float64)
    elif args.traj_source == "gnss":
        traj_df = valid_fix_df[["t_ns", "x", "y", "z"]].copy()
        traj_xyz = traj_df[["x", "y", "z"]].to_numpy(dtype=np.float64)
        traj_sat = valid_fix_df["sat_count"].to_numpy(dtype=np.float64)
    else:
        if not poses:
            raise ValueError(f"No pose samples read from topic {args.pose_topic}")
        poses_sorted = sorted(poses, key=lambda p: p.t_ns)
        traj_df = pd.DataFrame(
            [(p.t_ns, p.x, p.y, p.z) for p in poses_sorted],
            columns=["t_ns", "x", "y", "z"],
        )
        traj_xyz = traj_df[["x", "y", "z"]].to_numpy(dtype=np.float64)
        traj_sat = interpolate_knn(src_xyz, src_sat, traj_xyz, args.knn_k)

    if trajectory_vtk_in is None and args.traj_max_points > 0 and len(traj_xyz) > args.traj_max_points:
        keep = uniform_sample_indices(len(traj_xyz), args.traj_max_points)
        traj_df = traj_df.iloc[keep].reset_index(drop=True)
        traj_xyz = traj_xyz[keep]
        traj_sat = traj_sat[keep]

    # Map interpolation.
    if args.map_voxel_size > 0.0:
        map_xyz = (
            np.asarray(list(map_voxels.values()), dtype=np.float64)
            if map_voxels
            else np.empty((0, 3), dtype=np.float64)
        )
    else:
        map_xyz = np.asarray(map_points, dtype=np.float64) if map_points else np.empty((0, 3), dtype=np.float64)

    if not args.skip_map and len(map_xyz) > args.map_max_points:
        idx = np.arange(len(map_xyz))
        np.random.seed(args.seed)
        np.random.shuffle(idx)
        map_xyz = map_xyz[idx[: args.map_max_points]]

    map_sat = (
        interpolate_knn(src_xyz, src_sat, map_xyz, args.knn_k)
        if (not args.skip_map and len(map_xyz) > 0)
        else np.empty((0,), dtype=np.float64)
    )

    # Save CSV intermediates.
    valid_fix_df.to_csv(out_dir / "gnss_samples.csv", index=False)
    traj_df_out = traj_df.copy()
    traj_df_out["sat_count_mean"] = traj_sat
    traj_df_out.to_csv(out_dir / "trajectory_points.csv", index=False)

    if not args.skip_map and len(map_xyz) > 0:
        map_df = pd.DataFrame(map_xyz, columns=["x", "y", "z"])
        map_df["sat_count_mean"] = map_sat
        map_df.to_csv(out_dir / "map_points_sampled.csv", index=False)

    # Save ParaView files.
    if trajectory_vtk_in is None:
        write_vtp_polyline(
            out_dir / "trajectory_sat.vtp",
            traj_xyz,
            {"sat_count_mean": traj_sat},
        )
    else:
        out_vtk = trajectory_vtk_out if trajectory_vtk_out is not None else (out_dir / "trajectory_sat.vtk")
        write_colored_legacy_vtk_copy(trajectory_vtk_in, out_vtk, traj_sat)

    if not args.skip_map and len(map_xyz) > 0:
        write_vtp_points(
            out_dir / "map_sat.vtp",
            map_xyz,
            {"sat_count_mean": map_sat},
        )

    sat_stats = {
        "count": float(len(src_sat)),
        "mean": float(np.mean(src_sat)),
        "min": float(np.min(src_sat)),
        "max": float(np.max(src_sat)),
    }

    print("Done.")
    print(f"Output directory: {out_dir}")
    print(f"Matched GNSS samples: {int(sat_stats['count'])}")
    print(
        "Satellite count stats "
        f"(mean/min/max): {sat_stats['mean']:.2f}/{sat_stats['min']:.0f}/{sat_stats['max']:.0f}"
    )
    print(f"Trajectory points: {len(traj_sat)}")
    print(f"Map points (sampled): {len(map_xyz)}")


if __name__ == "__main__":
    main()
