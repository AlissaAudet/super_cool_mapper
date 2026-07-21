#!/usr/bin/env python3
"""Convert ROS1 .bag → ROS2 MCAP. Usage: convert_bag.py <in.bag> <out_dir> [foxglove|ros2]"""
import sys
import shutil
from pathlib import Path


def ros1_to_ros2_type(msgtype: str) -> str:
    """Convert ROS1 type string to ROS2 format."""
    parts = msgtype.split('/')
    if len(parts) == 2:
        return f'{parts[0]}/msg/{parts[1]}'
    return msgtype


def write_metadata(dst_dir: Path, mcap_name: str, stats: dict, serialization_format: str) -> None:
    """Write metadata.yaml for the ROS2 bag directory."""
    import yaml

    topics_with_count = []
    for topic, info in sorted(stats['topics'].items()):
        topics_with_count.append({
            'topic_metadata': {
                'name': topic,
                'type': ros1_to_ros2_type(info['type']),
                'serialization_format': serialization_format,
                'offered_qos_profiles': '',
            },
            'message_count': info['count'],
        })

    meta = {
        'rosbag2_bagfile_information': {
            'version': 9,
            'storage_identifier': 'mcap',
            'relative_file_paths': [mcap_name],
            'duration': {'nanoseconds': int(stats['duration_ns'])},
            'starting_time': {'nanoseconds_since_epoch': int(stats['start_ns'])},
            'message_count': stats['total'],
            'topics_with_message_count': topics_with_count,
            'compression_format': '',
            'compression_mode': '',
        }
    }

    (dst_dir / 'metadata.yaml').write_text(
        yaml.dump(meta, default_flow_style=False, allow_unicode=True)
    )



def convert_foxglove(src: Path, dst: Path) -> None:
    """ros1msg schema + ros1 encoding — fast, Foxglove-compatible."""
    from rosbags.rosbag1 import Reader
    from mcap.writer import Writer

    mcap_name = dst.name + '.mcap'
    mcap_path = dst / mcap_name

    stats = {'total': 0, 'start_ns': float('inf'), 'end_ns': 0,
             'duration_ns': 0, 'topics': {}}

    with Reader(src) as reader:
        connections = list(reader.connections)
        total = sum(c.msgcount for c in connections)
        print(f"Topics  : {len(set(c.topic for c in connections))} unique", flush=True)
        print(f"Messages: {total}", flush=True)

        with open(mcap_path, 'wb') as f:
            writer = Writer(f)
            writer.start(profile='', library='ros1-to-mcap-foxglove')
            try:
                schema_ids  = {}
                channel_ids = {}
                topic_types = {}

                for conn in connections:
                    if conn.msgtype not in schema_ids:
                        schema_ids[conn.msgtype] = writer.register_schema(
                            name=conn.msgtype,
                            encoding='ros1msg',
                            data=conn.msgdef.data.encode('utf-8', errors='replace'),
                        )
                    channel_ids[conn.id] = writer.register_channel(
                        topic=conn.topic,
                        message_encoding='ros1',
                        schema_id=schema_ids[conn.msgtype],
                    )
                    if conn.topic not in topic_types:
                        topic_types[conn.topic] = conn.msgtype

                print(f"Registered: {len(schema_ids)} schemas, {len(channel_ids)} channels", flush=True)

                count = 0
                for conn, ts, data in reader.messages():
                    if conn.id not in channel_ids:
                        continue
                    writer.add_message(channel_id=channel_ids[conn.id],
                                       log_time=ts, data=data, publish_time=ts)
                    count += 1
                    _update_stats(stats, ts, conn.topic, topic_types[conn.topic])
                    if count % 50_000 == 0:
                        pct = count * 100 // total if total else 0
                        print(f"Progress: {pct}% ({count}/{total})", flush=True)
            finally:
                writer.finish()

    stats['total'] = count
    _finalize_stats(stats)
    write_metadata(dst, mcap_name, stats, serialization_format='ros1')
    print(f"Done: {count} messages → {dst}/", flush=True)
    print(f"  {mcap_name}  +  metadata.yaml", flush=True)



def convert_ros2(src: Path, dst: Path) -> None:
    """Fully ROS2-compatible MCAP via rosbags-convert CLI."""
    import subprocess

    script_dir = Path(__file__).parent
    cli = script_dir.parent / '.venv' / 'bin' / 'rosbags-convert'
    if not cli.exists():
        cli = 'rosbags-convert'

    cmd = [str(cli), '--src', str(src), '--dst', str(dst), '--dst-storage', 'mcap']
    print(f"Running: {' '.join(cmd)}", flush=True)

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in proc.stdout:
        print(line.rstrip(), flush=True)
    proc.wait()

    if proc.returncode != 0:
        raise RuntimeError(f"rosbags-convert exited with code {proc.returncode}")

    print(f"Done → {dst}/", flush=True)

    _fix_metadata_for_humble(dst / 'metadata.yaml')
    mcap_files = list(dst.glob('*.mcap'))
    if mcap_files:
        _fix_mcap_embedded_metadata(mcap_files[0])


def _fix_metadata_for_humble(metadata_path: Path) -> None:
    """Rewrite metadata.yaml for ROS2 Humble compatibility."""
    import yaml as _yaml
    if not metadata_path.exists():
        return
    with open(metadata_path) as f:
        src_meta = _yaml.safe_load(f)
    if not src_meta or 'rosbag2_bagfile_information' not in src_meta:
        return
    src = src_meta['rosbag2_bagfile_information']

    clean_topics = []
    for t in src.get('topics_with_message_count', []):
        tm = t.get('topic_metadata', {})
        clean_topics.append({
            'topic_metadata': {
                'name': tm.get('name', ''),
                'type': tm.get('type', ''),
                'serialization_format': tm.get('serialization_format', 'cdr'),
                'offered_qos_profiles': '',
            },
            'message_count': t.get('message_count', 0),
        })

    clean = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': src.get('storage_identifier', 'mcap'),
            'duration': src.get('duration', {}),
            'starting_time': src.get('starting_time', {}),
            'message_count': src.get('message_count', 0),
            'topics_with_message_count': clean_topics,
            'compression_format': src.get('compression_format', ''),
            'compression_mode': src.get('compression_mode', ''),
            'relative_file_paths': src.get('relative_file_paths', []),
            'files': [],
        }
    }

    with open(metadata_path, 'w') as f:
        _yaml.dump(clean, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print("Fixed metadata.yaml for ROS2 Humble compatibility.", flush=True)


def _fix_mcap_embedded_metadata(mcap_path: Path) -> None:
    """Patch serialized_metadata YAML embedded in the MCAP binary."""
    import struct
    import mmap
    import yaml as _yaml

    # Use mmap for memory-efficient random access on large files
    with open(mcap_path, 'r+b') as f:
        mm = mmap.mmap(f.fileno(), 0)  # full file, read-write

        # Locate every occurrence of the 'serialized_metadata' key.
        # MCAP stores strings as: uint32_LE length + UTF-8 bytes.
        key_bytes = b'serialized_metadata'
        key_prefix = struct.pack('<I', len(key_bytes))
        search_pattern = key_prefix + key_bytes
        file_size = mm.size()

        patched = 0
        start = 0
        while True:
            idx = mm.find(search_pattern, start)
            if idx == -1:
                break

            val_len_offset = idx + len(search_pattern)
            if val_len_offset + 4 > file_size:
                break
            val_len = struct.unpack_from('<I', mm, val_len_offset)[0]
            val_offset = val_len_offset + 4
            val_end   = val_offset + val_len
            if val_end > file_size:
                break

            original_yaml = mm[val_offset:val_end].decode('utf-8', errors='replace')
            if 'files:' not in original_yaml and 'type_description_hash' not in original_yaml:
                start = val_end
                continue

            try:
                src = _yaml.safe_load(original_yaml)
                if not src or not isinstance(src, dict):
                    start = val_end
                    continue

                clean_topics = []
                for t in src.get('topics_with_message_count', []):
                    tm = t.get('topic_metadata', {})
                    clean_topics.append({
                        'topic_metadata': {
                            'name': tm.get('name', ''),
                            'type': tm.get('type', ''),
                            'serialization_format': tm.get('serialization_format', 'cdr'),
                            'offered_qos_profiles': '',
                        },
                        'message_count': t.get('message_count', 0),
                    })
                clean = {
                    'version': 5,
                    'storage_identifier': src.get('storage_identifier', 'mcap'),
                    'duration': src.get('duration', {}),
                    'starting_time': src.get('starting_time', {}),
                    'message_count': src.get('message_count', 0),
                    'topics_with_message_count': clean_topics,
                    'compression_format': src.get('compression_format', ''),
                    'compression_mode': src.get('compression_mode', ''),
                    'relative_file_paths': src.get('relative_file_paths', []),
                    'files': [],
                }
                fixed_yaml = _yaml.dump(clean, default_flow_style=False,
                                        allow_unicode=True, sort_keys=False)
                fixed_bytes = fixed_yaml.encode('utf-8')

                if len(fixed_bytes) <= val_len:
                    padded = fixed_bytes.ljust(val_len, b'\n')
                    mm[val_offset:val_end] = padded
                    patched += 1
                    print(f"Patched MCAP embedded metadata at offset {idx} "
                          f"({val_len} bytes).", flush=True)
                else:
                    print(f"WARNING: fixed YAML ({len(fixed_bytes)} B) > original "
                          f"({val_len} B) at {idx} — skipping.", flush=True)
            except Exception as exc:
                print(f"WARNING: could not patch at offset {idx}: {exc}", flush=True)

            start = val_end

        mm.flush()
        mm.close()

    if patched:
        print(f"Fixed embedded MCAP metadata ({patched} occurrence(s)).", flush=True)
    else:
        print("No embedded MCAP metadata to patch.", flush=True)



def _update_stats(stats, ts, topic, msgtype):
    """Update running stats with a new message timestamp and topic."""
    if ts < stats['start_ns']:
        stats['start_ns'] = ts
    if ts > stats['end_ns']:
        stats['end_ns'] = ts
    if topic not in stats['topics']:
        stats['topics'][topic] = {'type': msgtype, 'count': 0}
    stats['topics'][topic]['count'] += 1


def _finalize_stats(stats):
    """Finalize bag stats by computing duration from start/end timestamps."""
    if stats['start_ns'] == float('inf'):
        stats['start_ns'] = 0
    stats['duration_ns'] = max(0, stats['end_ns'] - stats['start_ns'])



def convert(src_path: str, dst_path: str, mode: str = 'foxglove') -> None:
    """Convert a ROS1 bag to a ROS2 bag directory in the given mode."""
    src = Path(src_path)
    dst = Path(dst_path)

    if dst.exists():
        shutil.rmtree(dst) if dst.is_dir() else dst.unlink()

    if mode == 'ros2':
        print("Mode: ROS2 CDR (full ros2 bag compatibility — re-serializing every message)", flush=True)
        dst.parent.mkdir(parents=True, exist_ok=True)
        convert_ros2(src, dst)
    else:
        print("Mode: Foxglove (ros1msg schema — fast copy)", flush=True)
        dst.mkdir(parents=True)
        convert_foxglove(src, dst)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.bag> <output_dir> [foxglove|ros2]", file=sys.stderr)
        sys.exit(1)
    mode = sys.argv[3] if len(sys.argv) > 3 else 'foxglove'
    try:
        convert(sys.argv[1], sys.argv[2], mode)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
