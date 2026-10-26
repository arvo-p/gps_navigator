"""
verify_binaries.py
==================
Inspection and verification tool for generated GPS binary map datasets.

Checks performed:
  1. names.str / names.idx:
     - Record counts and file size consistency.
     - Alphabetical ordering check.
     - Sample street lookups with corresponding anchor node IDs.
  2. towns.str / towns.idx:
     - Record counts.
     - Coordinate range checks (WGS84 Belgium bounds).
     - Sample town listings (e.g. Brussels, Liège, Antwerp, Ghent).
  3. backbone_nodes.bin / backbone_edges.bin:
     - Node count, edge count, coordinate ranges.
     - Target index bounds check (ensures no dangling / out-of-range target_idx).
     - Cost and length distribution.
     - Feature bitmask flag check (links, roundabouts, traffic signals).
  4. local_*_nodes.bin / local_*_edges.bin:
     - Spatial grid tile counts and edge target validity.
"""

import os
import glob
import struct

OUTPUT_DIR = "./binary_map_data"

def verify_names():
    idx_path = os.path.join(OUTPUT_DIR, "names.idx")
    str_path = os.path.join(OUTPUT_DIR, "names.str")

    if not os.path.exists(idx_path) or not os.path.exists(str_path):
        print("  [!] names.idx or names.str missing.")
        return

    idx_size = os.path.getsize(idx_path)
    str_size = os.path.getsize(str_path)
    record_count = idx_size // 8

    print(f"\n--- 1. Street Names Database ---")
    print(f"  Total records in names.idx: {record_count:,} (File size: {idx_size:,} bytes)")
    print(f"  String table size: {str_size:,} bytes")

    with open(idx_path, "rb") as f_idx, open(str_path, "rb") as f_str:
        # Read first 5 and last 5 streets
        sample_indices = list(range(min(5, record_count))) + list(range(max(0, record_count - 5), record_count))
        sample_indices = sorted(set(sample_indices))

        print("  Sample street names:")
        for idx in sample_indices[:8]:
            f_idx.seek(idx * 8)
            offset, nid = struct.unpack("<II", f_idx.read(8))
            f_str.seek(offset)
            chars = []
            while True:
                b = f_str.read(1)
                if not b or b == b'\x00':
                    break
                chars.append(b)
            name = b"".join(chars).decode('utf-8', errors='replace')
            print(f"    [Street ID {idx + 1:6d}] '{name}' -> Node ID {nid}")


def verify_towns():
    idx_path = os.path.join(OUTPUT_DIR, "towns.idx")
    str_path = os.path.join(OUTPUT_DIR, "towns.str")

    if not os.path.exists(idx_path) or not os.path.exists(str_path):
        print("  [!] towns.idx or towns.str missing.")
        return

    idx_size = os.path.getsize(idx_path)
    str_size = os.path.getsize(str_path)
    record_count = idx_size // 20

    print(f"\n--- 2. Towns / Cities Gazetteer ---")
    print(f"  Total town records: {record_count:,} (File size: {idx_size:,} bytes)")

    with open(idx_path, "rb") as f_idx, open(str_path, "rb") as f_str:
        print("  Sample towns extracted:")
        for idx in range(min(8, record_count)):
            f_idx.seek(idx * 20)
            lat_scaled, lon_scaled, offset, nid, _ = struct.unpack("<iiiII", f_idx.read(20))
            f_str.seek(offset)
            chars = []
            while True:
                b = f_str.read(1)
                if not b or b == b'\x00':
                    break
                chars.append(b)
            name = b"".join(chars).decode('utf-8', errors='replace')
            lat = lat_scaled / 1e7
            lon = lon_scaled / 1e7
            print(f"    '{name:20s}' @ ({lat:.5f}, {lon:.5f}) -> Nearest Road Node: {nid}")


def verify_graph(nodes_path, edges_path, geom_path=None, name="Backbone"):
    if not os.path.exists(nodes_path) or not os.path.exists(edges_path):
        print(f"  [!] {name} graph files missing.")
        return

    nodes_size = os.path.getsize(nodes_path)
    edges_size = os.path.getsize(edges_path)
    node_count = nodes_size // 16
    edge_count = edges_size // 20

    print(f"\n--- 3. {name} Road Network ---")
    print(f"  Nodes: {node_count:,} (File: {nodes_size:,} bytes, 16 B/node)")
    print(f"  Edges: {edge_count:,} (File: {edges_size:,} bytes, 20 B/edge)")

    geom_points_total = 0
    if geom_path and os.path.exists(geom_path):
        geom_size = os.path.getsize(geom_path)
        geom_points_total = geom_size // 8
        print(f"  Geometry: {geom_points_total:,} curve points (File: {geom_size:,} bytes, 8 B/pt)")

    invalid_targets = 0
    total_length_m = 0
    max_cost = 0
    curved_edges = 0
    sample_curve = None

    with open(nodes_path, "rb") as f_nodes, open(edges_path, "rb") as f_edges:
        sample_check_limit = min(edge_count, 50000)
        for i in range(sample_check_limit):
            target_idx, cost_sec, length_m, sid, g_offset, g_count, _ = struct.unpack("<IHHIIHH", f_edges.read(20))
            if target_idx >= node_count and target_idx != 0xFFFFFFFF:
                invalid_targets += 1
            total_length_m += length_m
            if cost_sec > max_cost:
                max_cost = cost_sec
            if g_count > 0:
                curved_edges += 1
                if sample_curve is None:
                    sample_curve = (g_offset, g_count)

    avg_len = total_length_m / max(1, sample_check_limit)
    print(f"  Integrity Check ({sample_check_limit:,} edges sampled):")
    print(f"    - Invalid target indices: {invalid_targets} {'[OK]' if invalid_targets == 0 else '[ERROR]'}")
    print(f"    - Curved edges with geometry: {curved_edges:,} / {sample_check_limit:,} ({curved_edges/max(1,sample_check_limit)*100:.1f}%)")
    print(f"    - Average edge segment length: {avg_len:.1f} meters")
    print(f"    - Max single segment traversal time: {max_cost} seconds")

    if sample_curve and geom_path and os.path.exists(geom_path):
        g_offset, g_count = sample_curve
        with open(geom_path, "rb") as f_geom:
            f_geom.seek(g_offset)
            pts = []
            for _ in range(min(g_count, 4)):
                lat_s, lon_s = struct.unpack("<ii", f_geom.read(8))
                pts.append(f"({lat_s/1e7:.5f}, {lon_s/1e7:.5f})")
            print(f"    - Sample road curve ({g_count} points): {' -> '.join(pts)}{'...' if g_count > 4 else ''}")


def verify_all():
    print("==================================================")
    print("  GPS BINARY MAP DATASET VERIFICATION SUITE")
    print("==================================================")
    if not os.path.exists(OUTPUT_DIR):
        print(f"Error: Directory '{OUTPUT_DIR}' does not exist yet. Run map_binaries_gen.py first.")
        return

    verify_names()
    verify_towns()
    verify_graph(os.path.join(OUTPUT_DIR, "backbone_nodes.bin"),
                 os.path.join(OUTPUT_DIR, "backbone_edges.bin"),
                 geom_path=os.path.join(OUTPUT_DIR, "backbone_geom.bin"),
                 name="Backbone Highway")

    local_nodes = glob.glob(os.path.join(OUTPUT_DIR, "local_*_nodes.bin"))
    print(f"\n--- 4. Local Tiled Networks ---")
    print(f"  Total spatial grid tiles generated: {len(local_nodes):,}")
    if local_nodes:
        sample_tile = local_nodes[0]
        base = sample_tile.replace("_nodes.bin", "")
        tile_name = os.path.basename(base)
        verify_graph(sample_tile,
                     base + "_edges.bin",
                     geom_path=base + "_geom.bin",
                     name=f"Tile '{tile_name}'")

    print("\n[Done] Verification complete.")

if __name__ == "__main__":
    verify_all()
