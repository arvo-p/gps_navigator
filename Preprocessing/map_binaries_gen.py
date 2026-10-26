"""
map_binaries_gen.py
===================
OpenStreetMap (OSM) Preprocessing Pipeline for Embedded GPS Navigation Systems.

Overview:
---------
This script parses raw OSM Protocol Buffer Format (.osm.pbf) map data and converts
it into highly compact, little-endian binary files optimized for resource-constrained
embedded systems (e.g. ARM microcontrollers, Raspberry Pi, or game engine simulations).

Architecture & Two-Pass Design:
-------------------------------
Raw OSM PBF files for an entire country (like Belgium) contain tens of millions of
nodes, taking hundreds of megabytes on disk and multiple gigabytes if loaded fully into
Python memory. To keep memory consumption minimal and avoid memory exhaustion:

  * Pass 1 (FirstPassHandler):
    - Traverses nodes to extract place/town labels and identify traffic features (signals, tolls).
    - Traverses ways to count node reference frequencies and record way endpoints.
    - Determines "Key Nodes" (road junctions, dead ends, and feature locations).
    - Discards shaping/geometry nodes from graph topology to perform edge contraction.

  * Pass 2 (WayDataCollector):
    - Traverses the PBF with 'locations=True' so node coordinates are cached by osmium's C++ engine.
    - Caches geographic coordinates for highway nodes.
    - Gathers parsed highway metadata (speed limits, oneway status, roundabout tags).

Output Binary Specifications:
-----------------------------
1. Street Names Database:
   - names.str : Null-terminated UTF-8 strings representing normalized unique street names.
   - names.idx : Binary index table (8 bytes per record, struct layout: '<II'):
                 [uint32 str_offset]  Byte offset into names.str.
                 [uint32 node_id]     OSM Node ID (lower 32-bits) located on this street.
                 * Sorted alphabetically by normalized name so client can binary-search.
                 * 1:1 mapped to edge 'street_id' (record index = street_id - 1).

2. Gazetteer / Towns Database:
   - towns.str : Null-terminated UTF-8 strings of town and city names.
   - towns.idx : Binary index table (20 bytes per record, struct layout: '<iiiII'):
                 [int32  lat_scaled]  Latitude scaled by 1e7 (fixed-point integer).
                 [int32  lon_scaled]  Longitude scaled by 1e7 (fixed-point integer).
                 [int32  str_offset]  Byte offset into towns.str.
                 [uint32 node_id]     Closest road graph node ID (snapped via spatial index).
                 [uint32 reserved]    Reserved padding for alignment/future flags (0).

3. Road Routing Graphs (Compressed Sparse Row / Forward-Star Format):
   Split into Backbone (motorways, trunks, primary highways) and Local (tiled by ~0.1 deg lat/lon):
   - *_nodes.bin (16 bytes per node, struct layout: '<iiIHH'):
                 [int32  lat_scaled]  Latitude * 1e7.
                 [int32  lon_scaled]  Longitude * 1e7.
                 [uint32 first_edge]  Index of first outgoing edge in the edges binary.
                 [uint16 edge_count]  Number of outgoing edges from this node.
                 [uint16 flags]       Feature bitmask (traffic light, roundabout, etc.).
   - *_edges.bin (20 bytes per edge, struct layout: '<IHHIIHH'):
                 [uint32 target_idx]  Index of target node in the corresponding *_nodes.bin.
                 [uint16 cost_sec]    Estimated traversal time in seconds (t = distance / speed).
                 [uint16 length_m]    Distance along road curves in meters.
                 [uint32 street_id]   1-based index into names.idx (0 if unnamed).
                 [uint32 geom_offset] Byte offset into corresponding *_geom.bin (0 if straight).
                 [uint16 geom_count]  Number of intermediate curve points (0 if straight).
                 [uint16 reserved]    Reserved for flags/alignment (0).
   - *_geom.bin (8 bytes per point, struct layout: '<ii'):
                 [int32  lat_scaled]  Latitude * 1e7.
                 [int32  lon_scaled]  Longitude * 1e7.
"""

import os
import re
import math
import struct
import unicodedata
from collections import defaultdict
import osmium

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

# Input OSM PBF file path
PBF_INPUT = "belgium-latest.osm.pbf"

# Output folder where all generated binary data and string tables will be stored
OUTPUT_DIR = "./binary_map_data"

# Geographic Bounding Box for Belgium: (min_lat, min_lon, max_lat, max_lon)
BBOX = (49.497, 2.544, 51.505, 6.408)

# Highway categorization:
# Car Backbone highways: Major transit corridors extracted globally for long-distance vehicle routing.
CAR_BACKBONE_HIGHWAYS = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link"
}
BACKBONE_HIGHWAYS = CAR_BACKBONE_HIGHWAYS

# Car Local highways: Slower/neighborhood roads, spatially partitioned into grid tiles.
CAR_LOCAL_HIGHWAYS = {
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "residential", "living_street",
    "unclassified", "service"
}
LOCAL_HIGHWAYS = CAR_LOCAL_HIGHWAYS

# Stroller Pedestrian highways: Dedicated footpaths, greenways, forest tracks, and trails.
STROLLER_PEDESTRIAN_HIGHWAYS = {
    "footway", "pedestrian",
    "path", "bridleway",
    "cycleway", "track",
    "steps"
}

# Stroller Backbone candidate highways: Nature trails, greenways, and quiet secondary/tertiary connectors.
STROLLER_BACKBONE_HIGHWAYS = {
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "cycleway", "track", "bridleway",
    "path"
}

# Combined set of all road and pedestrian way types processed by the pipeline
ALL_HIGHWAYS = CAR_BACKBONE_HIGHWAYS | CAR_LOCAL_HIGHWAYS | STROLLER_PEDESTRIAN_HIGHWAYS

# Default legal speed limits (km/h) across Belgium when OSM lacks an explicit 'maxspeed' tag.
SPEED_LIMITS = {
    "motorway": 120,
    "motorway_link": 70,
    "trunk": 90,
    "trunk_link": 60,
    "primary": 70,
    "primary_link": 50,
    "secondary": 70,
    "secondary_link": 50,
    "tertiary": 50,
    "tertiary_link": 40,
    "residential": 30,
    "living_street": 20,
    "unclassified": 50,
    "service": 15,
    "cycleway": 5,
    "track": 5,
    "footway": 5,
    "pedestrian": 5,
    "path": 5,
    "bridleway": 5,
    "steps": 5
}

# Bitmask flags encoded into node records (uint16) to inform routing and navigation engines
# about physical road features, mode eligibility, and backbone gateways.
FLAG_NONE             = 0x0000  # Default empty flag
FLAG_BACKBONE_LINK    = 0x0001  # Car gateway node connecting local road to car backbone
FLAG_TRAFFIC_LIGHT    = 0x0002  # Traffic signal intersection (adds delay in routing)
FLAG_BARRIER_GATE     = 0x0004  # Access gate, bollard, or physical barrier
FLAG_ROUNDABOUT       = 0x0008  # Node is part of a traffic roundabout
FLAG_TOLL_BOOTH       = 0x0010  # Toll collection booth
FLAG_BORDER_CROSSING  = 0x0020  # National border control or international crossing
FLAG_CAR_NODE         = 0x0040  # Node is accessible by car
FLAG_STROLLER_NODE    = 0x0080  # Node is accessible by stroller / pedestrian
FLAG_STROLLER_GATEWAY = 0x0100  # Stroller gateway node connecting local path/road to stroller backbone
FLAG_STROLLER_BARRIER = 0x0200  # Narrow barrier (turnstile, stile, kissing_gate) or raised curb

# Bitmask flags encoded into the 16-bit 'edge_flags' (formerly 'reserved') field of edge records:
#   - Low byte (bits 0..7): Mode access & backbone tier flags
#   - High byte (bits 8..15): Stroller convenience penalty multiplier in Q3 fixed-point (8 = 1.0x)
EDGE_CAR_ALLOWED       = 0x0001  # Edge is legal for cars in this direction
EDGE_STROLLER_ALLOWED  = 0x0002  # Edge is legal for strollers in this direction
EDGE_BOTH_ALLOWED      = 0x0003  # Edge is legal for both cars and strollers
EDGE_CAR_BACKBONE      = 0x0004  # Edge belongs to the car backbone tier
EDGE_STROLLER_BACKBONE = 0x0008  # Edge belongs to the stroller backbone tier


def classify_way(w):
    """
    Evaluates an OSM way and returns:
      (car_ok, stroller_ok, is_car_bb, is_stroller_bb_candidate, penalty_q3)
    where penalty_q3 is the stroller convenience multiplier in Q3 fixed-point (8 = 1.0x).
    """
    hw = w.tags.get("highway")
    if hw not in ALL_HIGHWAYS:
        return False, False, False, False, 8

    access = w.tags.get("access")
    service = w.tags.get("service")
    foot = w.tags.get("foot")
    motor_vehicle = w.tags.get("motor_vehicle")
    vehicle = w.tags.get("vehicle")

    # 1. Car accessibility check
    car_ok = (
        (hw in CAR_BACKBONE_HIGHWAYS or hw in CAR_LOCAL_HIGHWAYS)
        and access not in ("no", "private", "forestry", "agricultural")
        and motor_vehicle not in ("no", "private", "forestry", "agricultural")
        and vehicle not in ("no", "private", "forestry", "agricultural")
        and service not in ("parking_aisle", "driveway", "alley")
    )

    # 2. Stroller / Pedestrian accessibility check
    # Note: steps and rough tracks are NOT hard-excluded here so park & hillside
    # path networks (e.g. Parc de Cointe) are never severed into disconnected
    # islands; instead they receive a heavy convenience penalty (up to 31.8x)
    # so A* strongly avoids them whenever a smooth step-free detour exists.
    stroller_ok = True
    if hw in ("motorway", "motorway_link", "trunk", "trunk_link"):
        stroller_ok = False
    elif foot in ("no", "private"):
        stroller_ok = False
    elif access in ("no", "private") and foot not in ("yes", "designated", "permissive"):
        stroller_ok = False
    elif service in ("parking_aisle", "driveway"):
        stroller_ok = False

    if not car_ok and not stroller_ok:
        return False, False, False, False, 8

    is_car_bb = car_ok and (hw in CAR_BACKBONE_HIGHWAYS)

    # Stroller backbone candidates include regional roads (primary/secondary/tertiary),
    # greenway cycleways, and through-forest tracks/bridleways/paths (excluding urban
    # sidewalks, plazas, service alleys, and unclassified local streets which live in local_*_*.bin).
    footway_tag = w.tags.get("footway")
    is_stroller_bb_candidate = (
        stroller_ok
        and hw in STROLLER_BACKBONE_HIGHWAYS
        and footway_tag not in ("sidewalk", "crossing", "island", "traffic_island")
    )

    # 3. Calculate Stroller Convenience Penalty (Q3 fixed-point: 8 = 1.0x)
    penalty_q3 = 8
    if stroller_ok:
        surface = w.tags.get("surface", "")
        smoothness = w.tags.get("smoothness", "")
        tracktype = w.tags.get("tracktype", "")
        sidewalk = w.tags.get("sidewalk", "")
        has_ramp = (w.tags.get("ramp:stroller") == "yes" or w.tags.get("ramp") == "yes")

        # Surface factor
        if (
            surface in ("cobblestone", "unhewn_cobblestone", "grass", "dirt", "sand", "mud", "earth", "ground")
            or tracktype in ("grade3", "grade4", "grade5")
            or smoothness in ("bad", "very_bad", "horrible", "very_horrible", "impassable")
            or w.tags.get("sac_scale") is not None
        ):
            surf_mult = 4.5
        elif surface in ("sett", "gravel", "pebblestone", "unpaved"):
            surf_mult = 2.5
        elif tracktype == "grade2" or surface in ("wood", "metal") or smoothness == "intermediate":
            surf_mult = 1.35
        elif tracktype == "grade1" or surface in ("compacted", "fine_gravel"):
            surf_mult = 1.15
        else:
            surf_mult = 1.0

        # Highway comfort & sidewalk safety factor
        if hw in ("footway", "pedestrian", "living_street", "cycleway"):
            road_mult = 1.0
        elif hw in ("path", "bridleway", "track", "residential", "service"):
            road_mult = 1.1
        elif hw == "steps":
            road_mult = 2.0 if has_ramp else 16.0
        elif hw in ("unclassified", "tertiary", "tertiary_link"):
            road_mult = 2.5 if sidewalk in ("no", "none") else 1.25
        elif hw in ("secondary", "secondary_link", "primary", "primary_link"):
            road_mult = 2.5 if sidewalk in ("no", "none") else 1.5
        else:
            road_mult = 1.25

        if w.tags.get("stroller") == "no" or w.tags.get("wheelchair") == "no":
            road_mult *= 4.0

        penalty_q3 = min(255, max(8, int(round(8.0 * surf_mult * road_mult))))

    return car_ok, stroller_ok, is_car_bb, is_stroller_bb_candidate, penalty_q3


# ==============================================================================
# GEOMETRY & STRING UTILITIES
# ==============================================================================

def normalize_name(text):
    """
    Normalizes a street or town name for consistent sorting and searching.
    
    Operations:
      1. Applies Unicode NFKD decomposition to separate letters from diacritics/accents
         (e.g., 'é' -> 'e' + combining accent).
      2. Filters out combining diacritical marks (e.g., 'Liège' -> 'Liege').
      3. Converts to lowercase and strips non-alphanumeric characters.
      4. Collapses multiple whitespace characters into a single space.
    """
    if not text:
        return ""
    text = unicodedata.normalize('NFKD', text)
    text = "".join(c for c in text if not unicodedata.combining(c))
    text = text.lower().strip()
    text = re.sub(r'[^a-z0-9\s-]', '', text)
    text = re.sub(r'\s+', ' ', text)
    return text


def haversine_m(lat1, lon1, lat2, lon2):
    """
    Calculates the great-circle distance between two GPS coordinate points in meters.
    
    Formula:
      a = sin^2(dphi/2) + cos(phi1) * cos(phi2) * sin^2(dlambda/2)
      c = 2 * atan2(sqrt(a), sqrt(1-a))
      d = R * c
    Where:
      R = 6,371,000 meters (mean volumetric radius of Earth).
      Input latitudes and longitudes are given in decimal degrees.
    Includes clamping to [0.0, 1.0] to prevent domain errors with floating-point inaccuracies.
    """
    r = 6371000.0  # Earth mean radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    return 2.0 * r * math.asin(math.sqrt(max(0.0, min(1.0, a))))


# ==============================================================================
# OSM PROCESSING HANDLERS (PYOSMIUM)
# ==============================================================================

class FirstPassHandler(osmium.SimpleHandler):
    """
    First pass over the PBF file without storing node locations in memory.
    
    Objectives:
      1. Extract place points (cities, towns, villages) for the town gazetteer.
      2. Identify special node features (signals, barriers, tolls) and record bitflags.
      3. Count how many times each node ID appears across all drivable ways.
         - Nodes with ref count >= 2 are road junctions (where paths branch/cross).
         - Nodes that are way endpoints (first and last node) represent dead ends or terminals.
      4. These combined nodes form the 'Key Nodes' (vertices of the routing graph),
         allowing intermediate shaping nodes to be collapsed during edge contraction.
    """
    def __init__(self):
        super().__init__()
        # Maps node_id -> number of occurrences across highways
        self.node_ref_counts = defaultdict(int)
        # Maps node_id -> uint16 bitflags for special features
        self.special_nodes = {}
        # List of extracted towns: (node_id, lat, lon, name)
        self.town_nodes = []
        # Set of start and end nodes of every highway way (ensures cul-de-sacs are preserved)
        self.endpoint_nodes = set()

    def node(self, n):
        """Called for every OSM node entity during Pass 1."""
        if not n.location.valid():
            return

        # 1. Check if this node marks a populated place
        place = n.tags.get("place")
        if place in ["city", "town", "village", "municipality"]:
            name = n.tags.get("name")
            if name:
                self.town_nodes.append((n.id, n.location.lat, n.location.lon, name))

        # 2. Check for navigation-relevant feature tags
        flags = FLAG_NONE
        highway = n.tags.get("highway")
        barrier = n.tags.get("barrier")
        barrier_ext = n.tags.get("barrier:type")

        if highway == "traffic_signals":
            flags |= FLAG_TRAFFIC_LIGHT
        if barrier in ["gate", "lift_gate", "bollard"] or barrier_ext == "toll_booth":
            flags |= FLAG_BARRIER_GATE
        if barrier in ["turnstile", "stile", "kissing_gate", "squeeze", "log"] or n.tags.get("kerb") == "raised":
            flags |= FLAG_STROLLER_BARRIER
        if n.tags.get("barrier") == "toll_booth":
            flags |= FLAG_TOLL_BOOTH
        if n.tags.get("barrier") == "border_control" or n.tags.get("border_type") == "national":
            flags |= FLAG_BORDER_CROSSING

        if flags != FLAG_NONE:
            self.special_nodes[n.id] = flags

    def way(self, w):
        """Called for every OSM way entity during Pass 1."""
        if len(w.nodes) < 2:
            return
        car_ok, stroller_ok, _, _, _ = classify_way(w)
        if not car_ok and not stroller_ok:
            return

        # Record way boundaries so dead-end streets and road terminators are retained as vertices
        self.endpoint_nodes.add(w.nodes[0].ref)
        self.endpoint_nodes.add(w.nodes[-1].ref)

        # Count references for every node in the way using osmium's native iterator
        for n in w.nodes:
            self.node_ref_counts[n.ref] += 1


class WayDataCollector(osmium.SimpleHandler):
    """
    Second pass over the PBF file with 'locations=True'.
    
    Objectives:
      1. Cache geographic coordinates (lat, lon) for highway and pedestrian nodes.
      2. Parse highway speed limits (evaluating 'maxspeed' if present or falling back to defaults).
      3. Capture car and stroller directionality, access flags, stroller convenience penalties, and names.
    """
    def __init__(self, key_nodes):
        super().__init__()
        self.key_nodes = key_nodes
        self.parsed_ways = []
        # Maps node_id -> (lat, lon)
        self.node_coords = {}

    def node(self, n):
        """Called for nodes; caches key node coordinates."""
        if n.id in self.key_nodes and n.location.valid():
            self.node_coords[n.id] = (n.location.lat, n.location.lon)

    def way(self, w):
        """Called for ways; caches node coordinates along ways and parses highway attributes."""
        if len(w.nodes) < 2:
            return
        car_ok, stroller_ok, is_car_bb, is_stroller_bb_candidate, penalty_q3 = classify_way(w)
        if not car_ok and not stroller_ok:
            return

        hw = w.tags.get("highway")

        # Cache coordinates for all nodes in eligible ways so curving distance is accurately calculated
        for n in w.nodes:
            if n.ref not in self.node_coords and n.location.valid():
                self.node_coords[n.ref] = (n.location.lat, n.location.lon)

        name = w.tags.get("name", "")
        is_roundabout = w.tags.get("junction") == "roundabout"
        oneway_car = (w.tags.get("oneway") in ["yes", "1", "true"]) or is_roundabout
        oneway_stroller = w.tags.get("oneway:foot") in ["yes", "1", "true"]
        speed = SPEED_LIMITS.get(hw, 50)
        maxspeed = w.tags.get("maxspeed")
        if maxspeed and maxspeed.isdigit():
            speed = int(maxspeed)

        nodes = [n.ref for n in w.nodes]
        self.parsed_ways.append({
            "highway": hw,
            "name": name,
            "oneway_car": oneway_car,
            "oneway_stroller": oneway_stroller,
            "roundabout": is_roundabout,
            "car_ok": car_ok,
            "stroller_ok": stroller_ok,
            "is_car_bb": is_car_bb,
            "is_stroller_bb_candidate": is_stroller_bb_candidate,
            "penalty_q3": penalty_q3,
            "speed": max(5, speed),
            "nodes": nodes
        })


def prune_isolated_pedestrian_ways(parsed_ways):
    """
    Prunes pedestrian-only ways (ways where car_ok is False and stroller_ok is True)
    before building the local and backbone routing graphs:
      1. Splits pedestrian-only ways at junctions and road intersections into
         atomic sub-segments between endpoints.
      2. Anchor-Aware 2-Core Peeling: Iteratively removes dead-end pedestrian
         spurs (endpoint degree <= 1 that is NOT anchored to a car road node).
      3. Road-Anchored Component Filter: Discards any pedestrian component that
         does not connect at least 2 distinct car-road nodes (eliminating isolated
         garden/park islands and single-attachment loops).
    """
    car_ways = []
    ped_ways = []
    for w in parsed_ways:
        if w["car_ok"]:
            car_ways.append(w)
        elif w["stroller_ok"]:
            ped_ways.append(w)

    if not ped_ways:
        return car_ways

    # All node IDs present on the drivable road network
    road_nodes = set()
    for w in car_ways:
        for nid in w["nodes"]:
            road_nodes.add(nid)

    # Count node degrees across pedestrian-only ways to split at internal junctions
    ped_node_deg = defaultdict(int)
    for w in ped_ways:
        nodes = w["nodes"]
        for i, nid in enumerate(nodes):
            if i == 0 or i == len(nodes) - 1:
                ped_node_deg[nid] += 1
            else:
                ped_node_deg[nid] += 2

    # Split pedestrian ways at internal junctions or road connection points
    sub_ways = []
    for w in ped_ways:
        nodes = w["nodes"]
        if len(nodes) < 2:
            continue
        start_idx = 0
        for i in range(1, len(nodes)):
            nid = nodes[i]
            is_end = (i == len(nodes) - 1)
            is_junction = (ped_node_deg[nid] != 2) or (nid in road_nodes)
            if is_end or is_junction:
                sub_nodes = nodes[start_idx : i + 1]
                if len(sub_nodes) >= 2:
                    sub_w = dict(w)
                    sub_w["nodes"] = sub_nodes
                    sub_ways.append(sub_w)
                start_idx = i

    if not sub_ways:
        return car_ways

    adj_segs = defaultdict(set)
    seg_endpoints = []
    for idx, sw in enumerate(sub_ways):
        u = sw["nodes"][0]
        v = sw["nodes"][-1]
        seg_endpoints.append((u, v))
        adj_segs[u].add(idx)
        adj_segs[v].add(idx)

    # Step 1: Iteratively peel dead-end pedestrian spurs not anchored to road_nodes
    removed_segs = set()
    queue = [
        nid for nid, segs in adj_segs.items()
        if nid not in road_nodes and len(segs) <= 1
    ]
    while queue:
        curr = queue.pop()
        if curr in road_nodes:
            continue
        if len(adj_segs[curr]) > 1:
            continue
        for seg_idx in list(adj_segs[curr]):
            if seg_idx in removed_segs:
                continue
            removed_segs.add(seg_idx)
            u, v = seg_endpoints[seg_idx]
            adj_segs[u].discard(seg_idx)
            adj_segs[v].discard(seg_idx)
            other = v if u == curr else u
            if other not in road_nodes and len(adj_segs[other]) <= 1:
                queue.append(other)

    # Step 2: Keep only components touching at least 2 distinct car-road nodes
    visited_pts = set()
    kept_segs = set()
    for start_pt in list(adj_segs.keys()):
        if start_pt in visited_pts or not adj_segs[start_pt]:
            continue
        comp_pts = set()
        comp_segs = set()
        comp_road_pts = set()
        bfs_q = [start_pt]
        visited_pts.add(start_pt)
        while bfs_q:
            curr = bfs_q.pop()
            comp_pts.add(curr)
            if curr in road_nodes:
                comp_road_pts.add(curr)
            for seg_idx in adj_segs[curr]:
                if seg_idx in removed_segs:
                    continue
                comp_segs.add(seg_idx)
                u, v = seg_endpoints[seg_idx]
                for nxt in (u, v):
                    if nxt not in visited_pts:
                        visited_pts.add(nxt)
                        bfs_q.append(nxt)

        if len(comp_road_pts) >= 2:
            kept_segs.update(comp_segs)

    surviving_ped_ways = [
        sw for idx, sw in enumerate(sub_ways)
        if idx in kept_segs
    ]
    print(f"      Pruned pedestrian ways: {len(ped_ways):,} raw -> {len(surviving_ped_ways):,} connected through-segments kept.")
    return car_ways + surviving_ped_ways


def simplify_geom_points(u_coord, v_coord, geom_points, tol_m=1.5):
    """
    Simplifies intermediate shaping points along an edge (u_coord -> geom_points -> v_coord)
    using iterative Ramer-Douglas-Peucker (RDP) with a perpendicular tolerance of tol_m meters
    (default 1.5 m, which is < 1 pixel even at Zoom 16).
    Drops redundant collinear GPS points along straight sections while preserving all curves.
    """
    n = len(geom_points)
    if n <= 1:
        return geom_points

    pts = [u_coord] + list(geom_points) + [v_coord]
    total = len(pts)
    keep = [False] * total
    keep[0] = True
    keep[total - 1] = True

    # Local flat-Earth metric scaling around Belgium (~50.5° N)
    m_per_lat = 111320.0
    m_per_lon = 70500.0
    tol_sq = tol_m * tol_m

    stack = [(0, total - 1)]
    while stack:
        i, j = stack.pop()
        if j - i <= 1:
            continue

        ax, ay = pts[i][1] * m_per_lon, pts[i][0] * m_per_lat
        bx, by = pts[j][1] * m_per_lon, pts[j][0] * m_per_lat
        dx, dy = bx - ax, by - ay
        len_sq = dx * dx + dy * dy

        max_d_sq = -1.0
        max_k = -1
        for k in range(i + 1, j):
            px, py = pts[k][1] * m_per_lon, pts[k][0] * m_per_lat
            if len_sq <= 1e-6:
                d_sq = (px - ax) ** 2 + (py - ay) ** 2
            else:
                t = ((px - ax) * dx + (py - ay) * dy) / len_sq
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                proj_x = ax + t * dx
                proj_y = ay + t * dy
                d_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2

            if d_sq > max_d_sq:
                max_d_sq = d_sq
                max_k = k

        if max_d_sq > tol_sq and max_k != -1:
            keep[max_k] = True
            stack.append((i, max_k))
            stack.append((max_k, j))

    return [pts[k] for k in range(1, total - 1) if keep[k]]


def prune_stroller_backbone(adj_map, node_coords):
    """
    Prunes and compresses the Stroller Backbone adjacency map so it contains ONLY
    continuous inter-town and through-nature routes with minimal RAM/disk footprint:
      1. Series-Parallel Reduction (3 rounds):
         - Deduplicates parallel edges/loops between the same pair of vertices.
         - Iterative 2-Core Dead-End Peeling: Strips degree-1 chains (dead-end
           forest spurs and cul-de-sacs).
         - Degree-2 Pass-Through Node Contraction: Contracts degree-2 vertices
           (u <-> w <-> v into u <-> v up to 1500 m per edge) while moving w's
           coordinate into geom_points so the drawn curve is 100% preserved.
      2. Giant Connected Component (LCC) Filter: Runs BFS and discards isolated
         park loops or disconnected trail islands.
      3. Lossless Collinear Geometry Simplification (<= 1.5 m RDP tolerance):
         Removes redundant collinear GPS points on straight segments while
         preserving every turn and curve along the road/trail.
    """
    def edge_stroller_cost(e):
        p = max(8, e.get("edge_flags", 0) >> 8)
        return max(1, e["length_m"]) * p

    def dedup_edges(amap):
        for u in list(amap.keys()):
            best = {}
            for e in amap[u]:
                v = e["target"]
                if v == u:
                    continue
                c = edge_stroller_cost(e)
                if v not in best or c < edge_stroller_cost(best[v]):
                    best[v] = e
            if best:
                amap[u] = list(best.values())
            else:
                del amap[u]

    def peel_2core(amap):
        nbrs = defaultdict(set)
        for u, elist in amap.items():
            for e in elist:
                v = e["target"]
                if u != v:
                    nbrs[u].add(v)
                    nbrs[v].add(u)
        q = [n for n, s in nbrs.items() if len(s) <= 1]
        dead = set()
        while q:
            curr = q.pop()
            if curr in dead:
                continue
            dead.add(curr)
            for nb in list(nbrs[curr]):
                nbrs[nb].discard(curr)
                if len(nbrs[nb]) <= 1 and nb not in dead:
                    q.append(nb)
        for u in list(amap.keys()):
            if u in dead:
                del amap[u]
            else:
                elist = [e for e in amap[u] if e["target"] not in dead]
                if elist:
                    amap[u] = elist
                else:
                    del amap[u]

    def merge_two_edges(e1, e2, w_coord):
        l1, l2 = e1["length_m"], e2["length_m"]
        new_len = min(65535, l1 + l2)
        new_cost = min(65535, e1["cost"] + e2["cost"])
        p1 = max(8, e1.get("edge_flags", 0) >> 8)
        p2 = max(8, e2.get("edge_flags", 0) >> 8)
        tot_l = max(1, l1 + l2)
        new_p = min(255, max(8, int(round((p1 * max(1, l1) + p2 * max(1, l2)) / tot_l))))
        low_mask = (e1.get("edge_flags", 0) & e2.get("edge_flags", 0)) & 0x00FF
        return {
            "target": e2["target"],
            "cost": new_cost,
            "length_m": new_len,
            "street_id": e1["street_id"] or e2["street_id"],
            "geom_points": e1["geom_points"] + [w_coord] + e2["geom_points"],
            "edge_flags": low_mask | (new_p << 8)
        }

    def contract_deg2(amap, max_len=1500):
        nbrs = defaultdict(set)
        out_edges = defaultdict(dict)
        for u, elist in amap.items():
            for e in elist:
                v = e["target"]
                if u != v:
                    nbrs[u].add(v)
                    nbrs[v].add(u)
                    out_edges[u][v] = e

        candidates = [w for w, s in nbrs.items() if len(s) == 2]
        for w in candidates:
            if len(nbrs[w]) != 2 or w not in node_coords:
                continue
            u, v = tuple(nbrs[w])
            if u == v:
                continue
            e_uw = out_edges[u].get(w)
            e_wv = out_edges[w].get(v)
            e_vw = out_edges[v].get(w)
            e_wu = out_edges[w].get(u)
            if not ((e_uw and e_wv) or (e_vw and e_wu)):
                continue
            if e_uw and e_wv and (e_uw["length_m"] + e_wv["length_m"] > max_len):
                continue
            if e_vw and e_wu and (e_vw["length_m"] + e_wu["length_m"] > max_len):
                continue

            w_coord = node_coords[w]
            out_edges[u].pop(w, None)
            out_edges[v].pop(w, None)
            out_edges.pop(w, None)
            nbrs[u].discard(w)
            nbrs[v].discard(w)
            del nbrs[w]

            if e_uw and e_wv:
                m_uv = merge_two_edges(e_uw, e_wv, w_coord)
                prev = out_edges[u].get(v)
                if not prev or edge_stroller_cost(m_uv) < edge_stroller_cost(prev):
                    out_edges[u][v] = m_uv
                nbrs[u].add(v)
                nbrs[v].add(u)

            if e_vw and e_wu:
                m_vu = merge_two_edges(e_vw, e_wu, w_coord)
                prev = out_edges[v].get(u)
                if not prev or edge_stroller_cost(m_vu) < edge_stroller_cost(prev):
                    out_edges[v][u] = m_vu
                nbrs[u].add(v)
                nbrs[v].add(u)

        amap.clear()
        for u, d in out_edges.items():
            if d:
                amap[u] = list(d.values())

    # Step 1: 3 rounds of Series-Parallel Reduction (dedup + 2-core peel + deg-2 contraction)
    for _ in range(3):
        dedup_edges(adj_map)
        peel_2core(adj_map)
        contract_deg2(adj_map, max_len=1500)
    dedup_edges(adj_map)
    peel_2core(adj_map)

    # Step 2: Find the single Giant Connected Component (LCC) via BFS
    neighbors = defaultdict(set)
    for u, edges in adj_map.items():
        for e in edges:
            v = e["target"]
            if u != v:
                neighbors[u].add(v)
                neighbors[v].add(u)

    visited = set()
    largest_cc = set()
    for start_node in neighbors:
        if start_node in visited or not neighbors[start_node]:
            continue
        comp = set()
        bfs_q = [start_node]
        visited.add(start_node)
        while bfs_q:
            curr = bfs_q.pop()
            comp.add(curr)
            for nbr in neighbors[curr]:
                if nbr not in visited:
                    visited.add(nbr)
                    bfs_q.append(nbr)
        if len(comp) > len(largest_cc):
            largest_cc = comp

    # Step 3: Rebuild pruned adjacency map and simplify collinear curve points (<= 1.5 m RDP)
    pruned = defaultdict(list)
    for u in largest_cc:
        if u in adj_map and u in node_coords:
            u_coord = node_coords[u]
            kept_edges = []
            for e in adj_map[u]:
                v = e["target"]
                if v in largest_cc and v in node_coords:
                    e_copy = dict(e)
                    if len(e_copy["geom_points"]) > 1:
                        e_copy["geom_points"] = simplify_geom_points(
                            u_coord, node_coords[v], e_copy["geom_points"], tol_m=1.5
                        )
                    kept_edges.append(e_copy)
            if kept_edges:
                pruned[u] = kept_edges

    return pruned, largest_cc


# ==============================================================================
# PIPELINE EXECUTION & GRAPH GENERATION
# ==============================================================================

def run_pipeline():
    """Main execution entry point: orchestrates parsing, graph building, and binary export."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # --------------------------------------------------------------------------
    # STAGE 1: Pass 1 Analysis
    # --------------------------------------------------------------------------
    print("[1/5] Running Pass 1: Scanning road topology, junctions, and towns...")
    pass1 = FirstPassHandler()
    pass1.apply_file(PBF_INPUT, locations=False)

    # Determine graph vertices ("Key Nodes"):
    # - Start and end nodes of any way (cul-de-sacs, road boundaries)
    # - Intersections where 2 or more ways cross/branch (count >= 2)
    # - Nodes containing special navigation features (traffic lights, tolls, etc.)
    key_nodes = set(pass1.endpoint_nodes)
    for nid, count in pass1.node_ref_counts.items():
        if count >= 2 or nid in pass1.special_nodes:
            key_nodes.add(nid)

    print(f"      Total unique road nodes identified: {len(pass1.node_ref_counts):,}")
    print(f"      Total graph vertices (key nodes) selected: {len(key_nodes):,}")

    # --------------------------------------------------------------------------
    # STAGE 2: Pass 2 Data Extraction
    # --------------------------------------------------------------------------
    print("[2/5] Running Pass 2: Extracting coordinates and highway metadata...")
    pass2 = WayDataCollector(key_nodes)
    pass2.apply_file(PBF_INPUT, locations=True)
    print(f"      Parsed {len(pass2.parsed_ways):,} ways. Cached {len(pass2.node_coords):,} node coordinates.")

    # Prune isolated & dead-end pedestrian ways so local tiles and stroller backbone
    # only contain connected pedestrian through-paths anchored to the road network
    pass2.parsed_ways = prune_isolated_pedestrian_ways(pass2.parsed_ways)

    # --------------------------------------------------------------------------
    # STAGE 3: Street Names Indexing
    # --------------------------------------------------------------------------
    print("[3/5] Indexing and sorting street names...")
    # Gather unique street names and pick a representative graph node ID on that street
    street_info = {}
    for way in pass2.parsed_ways:
        name = way["name"]
        if name:
            norm = normalize_name(name)
            if norm and norm not in street_info:
                sample_nid = way["nodes"][0] if way["nodes"] else 0
                street_info[norm] = (name, sample_nid)

    # Sort alphabetically by normalized name so names.idx is binary-searchable by string
    sorted_norm_names = sorted(street_info.keys())
    # 1-based street ID: street_id directly matches record index (street_id - 1) in names.idx
    street_name_to_id = {}
    for idx, norm in enumerate(sorted_norm_names):
        sid = idx + 1
        raw_name, sample_nid = street_info[norm]
        street_name_to_id[norm] = (sid, raw_name, sample_nid)

    def get_street_id(raw_name):
        """Looks up the deterministic 1-based street_id for a given raw name."""
        norm = normalize_name(raw_name)
        if norm in street_name_to_id:
            return street_name_to_id[norm][0]
        return 0

    # --------------------------------------------------------------------------
    # STAGE 4: Graph Topology Construction & Edge Contraction
    # --------------------------------------------------------------------------
    print("[4/5] Building compressed graph edges (contracting intermediate nodes)...")
    backbone_adjacency = defaultdict(list)
    stroller_backbone_adjacency = defaultdict(list)
    local_adjacency = defaultdict(lambda: defaultdict(list))
    all_graph_nodes = set()
    car_backbone_nodes = set()
    local_graph_nodes = set()
    node_flags = defaultdict(int)

    # Propagate POI / special flags to nodes that have valid coordinates
    for nid, f in pass1.special_nodes.items():
        if nid in pass2.node_coords:
            node_flags[nid] |= f

    for way in pass2.parsed_ways:
        hw = way["highway"]
        car_ok = way["car_ok"]
        stroller_ok = way["stroller_ok"]
        is_car_bb = way["is_car_bb"]
        is_stroller_bb_cand = way["is_stroller_bb_candidate"]
        penalty_q3 = way["penalty_q3"]
        speed = way["speed"]
        nodes = way["nodes"]
        is_roundabout = way["roundabout"]
        is_link = "_link" in hw
        street_id = get_street_id(way["name"]) if way["name"] else 0

        # Local tiles contain all non-car-backbone ways, plus primary roads where strollers are allowed
        in_local_tile = (not is_car_bb) or stroller_ok

        # Edge Contraction Loop:
        # Walk along the way's node sequence. A new edge starts at 'u' (a key node)
        # and stops at the next key node 'v'. All intermediate shaping nodes accumulate
        # distance along curves, compressing the graph significantly.
        segment_start_idx = 0
        while segment_start_idx < len(nodes) - 1:
            u = nodes[segment_start_idx]
            accumulated_m = 0.0
            segment_end_idx = segment_start_idx + 1

            # Accumulate Haversine distance along intermediate geometry until hitting a key node
            while segment_end_idx < len(nodes):
                curr = nodes[segment_end_idx - 1]
                nxt = nodes[segment_end_idx]
                if curr in pass2.node_coords and nxt in pass2.node_coords:
                    c1 = pass2.node_coords[curr]
                    c2 = pass2.node_coords[nxt]
                    accumulated_m += haversine_m(c1[0], c1[1], c2[0], c2[1])

                if nxt in key_nodes:
                    break
                segment_end_idx += 1

            if segment_end_idx >= len(nodes):
                break

            v = nodes[segment_end_idx]

            # Only construct edges if both vertices have valid coordinates
            if u in pass2.node_coords and v in pass2.node_coords:
                all_graph_nodes.add(u)
                all_graph_nodes.add(v)

                # Set mode eligibility and structural maneuver flags on nodes
                if car_ok:
                    node_flags[u] |= FLAG_CAR_NODE
                    node_flags[v] |= FLAG_CAR_NODE
                if stroller_ok:
                    node_flags[u] |= FLAG_STROLLER_NODE
                    node_flags[v] |= FLAG_STROLLER_NODE
                if is_link and car_ok:
                    node_flags[u] |= FLAG_BACKBONE_LINK
                    node_flags[v] |= FLAG_BACKBONE_LINK
                if is_roundabout:
                    node_flags[u] |= FLAG_ROUNDABOUT
                    node_flags[v] |= FLAG_ROUNDABOUT

                # Straight-line fallback if intermediate nodes were missing
                if accumulated_m == 0.0:
                    c1 = pass2.node_coords[u]
                    c2 = pass2.node_coords[v]
                    accumulated_m = haversine_m(c1[0], c1[1], c2[0], c2[1])

                # Calculate length (meters) and car traversal cost (seconds)
                length_m = min(65535, int(round(accumulated_m)))
                cost_sec = min(65535, max(1, int(round(accumulated_m / (speed * 0.27778)))))

                # Extract intermediate shaping coordinates for smooth curve rendering
                intermediate_nodes = nodes[segment_start_idx + 1 : segment_end_idx]
                geom_points = [
                    pass2.node_coords[nid]
                    for nid in intermediate_nodes
                    if nid in pass2.node_coords
                ]

                # Build forward edge access mask (u -> v)
                fwd_mask = 0
                if car_ok:
                    fwd_mask |= EDGE_CAR_ALLOWED
                    if is_car_bb:
                        fwd_mask |= EDGE_CAR_BACKBONE
                if stroller_ok:
                    fwd_mask |= EDGE_STROLLER_ALLOWED
                    if is_stroller_bb_cand:
                        fwd_mask |= EDGE_STROLLER_BACKBONE

                fwd_flags = fwd_mask | ((penalty_q3 & 0xFF) << 8)

                edge_forward = {
                    "target": v,
                    "cost": cost_sec,
                    "length_m": length_m,
                    "street_id": street_id,
                    "geom_points": geom_points,
                    "edge_flags": fwd_flags
                }

                if is_car_bb:
                    backbone_adjacency[u].append(edge_forward)
                    car_backbone_nodes.add(u)
                    car_backbone_nodes.add(v)

                if is_stroller_bb_cand:
                    stroller_backbone_adjacency[u].append(dict(edge_forward))

                if in_local_tile:
                    # In local tiles, if this is a primary road that is car-backbone, keep car/stroller flags
                    u_lat, u_lon = pass2.node_coords[u]
                    grid_key = f"{int(u_lat*10):03d}_{int(u_lon*10):03d}"
                    local_adjacency[grid_key][u].append(edge_forward)
                    local_graph_nodes.add(u)
                    local_graph_nodes.add(v)

                # Build backward edge access mask (v -> u)
                bwd_car_ok = car_ok and not way["oneway_car"]
                bwd_stroller_ok = stroller_ok and not way["oneway_stroller"]
                bwd_mask = 0
                if bwd_car_ok:
                    bwd_mask |= EDGE_CAR_ALLOWED
                    if is_car_bb:
                        bwd_mask |= EDGE_CAR_BACKBONE
                if bwd_stroller_ok:
                    bwd_mask |= EDGE_STROLLER_ALLOWED
                    if is_stroller_bb_cand:
                        bwd_mask |= EDGE_STROLLER_BACKBONE

                if bwd_mask != 0:
                    bwd_flags = bwd_mask | ((penalty_q3 & 0xFF) << 8)
                    edge_backward = {
                        "target": u,
                        "cost": cost_sec,
                        "length_m": length_m,
                        "street_id": street_id,
                        "geom_points": geom_points[::-1],
                        "edge_flags": bwd_flags
                    }
                    if is_car_bb and bwd_car_ok:
                        backbone_adjacency[v].append(edge_backward)
                    if is_stroller_bb_cand and bwd_stroller_ok:
                        stroller_backbone_adjacency[v].append(dict(edge_backward))
                    if in_local_tile:
                        v_lat, v_lon = pass2.node_coords[v]
                        v_grid_key = f"{int(v_lat*10):03d}_{int(v_lon*10):03d}"
                        local_adjacency[v_grid_key][v].append(edge_backward)

            segment_start_idx = segment_end_idx

    # Prune & compress Stroller Backbone: 2-core peeling + deg-2 contraction + Giant Component + 1.5m RDP
    print("      Pruning & contracting stroller backbone (2-core + degree-2 contraction + Giant Component)...")
    stroller_backbone_adjacency, stroller_bb_nodes = prune_stroller_backbone(
        stroller_backbone_adjacency, pass2.node_coords
    )
    print(f"      Stroller backbone vertices retained in Giant Component: {len(stroller_bb_nodes):,}")

    # Flag gateway intersections connecting local tiles to Car Backbone and Stroller Backbone
    for nid in (local_graph_nodes & car_backbone_nodes):
        node_flags[nid] |= FLAG_BACKBONE_LINK
    for nid in (local_graph_nodes & stroller_bb_nodes):
        node_flags[nid] |= FLAG_STROLLER_GATEWAY

    # --------------------------------------------------------------------------
    # STAGE 5: Export Binary Files
    # --------------------------------------------------------------------------
    print("[5/5] Exporting binary map datasets and string tables...")

    # 1. Export Street Names Database
    # names.str : Null-terminated strings
    # names.idx : Binary records <II (8 bytes each): [uint32 offset, uint32 node_id]
    str_blob = bytearray()
    idx_records = bytearray()
    for norm in sorted_norm_names:
        sid, raw_name, sample_nid = street_name_to_id[norm]
        offset = len(str_blob)
        encoded_str = raw_name.encode('utf-8') + b'\x00'
        str_blob.extend(encoded_str)
        idx_records.extend(struct.pack("<II", offset, sample_nid & 0xFFFFFFFF))

    with open(os.path.join(OUTPUT_DIR, "names.str"), "wb") as f:
        f.write(str_blob)
    with open(os.path.join(OUTPUT_DIR, "names.idx"), "wb") as f:
        f.write(idx_records)

    # 2. Build 2D Spatial Grid Index for Fast Town-to-Road Snapping
    # Cells are binned by 0.1 degree latitude and longitude (~11 km x 7 km).
    spatial_grid = defaultdict(list)
    for gn in all_graph_nodes:
        if gn in pass2.node_coords:
            glat, glon = pass2.node_coords[gn]
            spatial_grid[(int(glat * 10), int(glon * 10))].append((gn, glat, glon))

    def find_closest_graph_node(lat, lon):
        """Finds the closest road graph node to a town center using the spatial grid."""
        g_lat_idx = int(lat * 10)
        g_lon_idx = int(lon * 10)
        best_d = float('inf')
        best_nid = 0
        # Check current cell and all 8 neighboring cells
        for dlat in (-1, 0, 1):
            for dlon in (-1, 0, 1):
                cell = spatial_grid.get((g_lat_idx + dlat, g_lon_idx + dlon))
                if cell:
                    for gn, clat, clon in cell:
                        d = (clat - lat)**2 + (clon - lon)**2
                        if d < best_d:
                            best_d = d
                            best_nid = gn
        # Fallback search if cell is sparse/empty
        if best_nid == 0 and all_graph_nodes:
            sample = list(all_graph_nodes)[:1000]
            for gn in sample:
                if gn in pass2.node_coords:
                    clat, clon = pass2.node_coords[gn]
                    d = (clat - lat)**2 + (clon - lon)**2
                    if d < best_d:
                        best_d = d
                        best_nid = gn
        return best_nid

    # 3. Export Towns Gazetteer
    # towns.str : Null-terminated town strings
    # towns.idx : Binary records <iiiII (20 bytes each):
    #             [lat*1e7, lon*1e7, str_offset, node_id, reserved=0]
    town_blob = bytearray()
    town_idx = bytearray()
    for tid, tlat, tlon, tname in sorted(pass1.town_nodes, key=lambda x: x[3].lower()):
        t_offset = len(town_blob)
        town_blob.extend(tname.encode('utf-8') + b'\x00')
        node_id_close = find_closest_graph_node(tlat, tlon)
        town_idx.extend(struct.pack("<iiiII", int(tlat * 1e7), int(tlon * 1e7), t_offset, node_id_close & 0xFFFFFFFF, 0))

    with open(os.path.join(OUTPUT_DIR, "towns.str"), "wb") as f:
        f.write(town_blob)
    with open(os.path.join(OUTPUT_DIR, "towns.idx"), "wb") as f:
        f.write(town_idx)

    # 4. Export Road Graphs in Compressed Sparse Row (CSR) Format
    def write_graph_binary(adj_map, nodes_bin_path, edges_bin_path, geom_bin_path):
        """
        Serializes an adjacency map into contiguous binary files:
        
        *_nodes.bin (16 bytes per node, struct '<iiIHH'):
          [int32  lat_scaled] : latitude * 1e7
          [int32  lon_scaled] : longitude * 1e7
          [uint32 first_edge] : offset index into the edge file
          [uint16 edge_count] : number of outgoing edges
          [uint16 flags]      : feature bitmask

        *_edges.bin (20 bytes per edge, struct '<IHHIIHH'):
          [uint32 target_idx] : target node index in *_nodes.bin
          [uint16 cost_sec]   : travel time in seconds
          [uint16 length_m]   : distance in meters
          [uint32 street_id]  : 1-based index in names.idx
          [uint32 geom_offset]: byte offset in *_geom.bin (0 if straight)
          [uint16 geom_count] : number of intermediate curve points
          [uint16 reserved]   : 0

        *_geom.bin (8 bytes per point, struct '<ii'):
          [int32  lat_scaled] : latitude * 1e7
          [int32  lon_scaled] : longitude * 1e7
        """
        # Collect all nodes appearing as a source OR target
        all_nodes = set(adj_map.keys())
        for edges in adj_map.values():
            for e in edges:
                all_nodes.add(e["target"])

        # Filter nodes with valid coordinates and assign local contiguous indices [0 .. N-1]
        unique_nodes = [nid for nid in all_nodes if nid in pass2.node_coords]
        node_to_idx = {nid: idx for idx, nid in enumerate(unique_nodes)}

        node_records = bytearray()
        edge_records = bytearray()
        geom_records = bytearray()
        curr_edge_idx = 0

        for nid in unique_nodes:
            edges = adj_map.get(nid, [])
            lat, lon = pass2.node_coords[nid]
            lat_scaled = int(round(lat * 1e7))
            lon_scaled = int(round(lon * 1e7))
            first_edge = curr_edge_idx
            edge_count = len(edges)
            flags = node_flags[nid]

            # Write node record (16 bytes)
            node_records.extend(struct.pack("<iiIHH", lat_scaled, lon_scaled, first_edge, edge_count, flags))

            # Write outgoing edge records (20 bytes each) and geometry curve points (8 bytes each)
            for e in edges:
                target_idx = node_to_idx.get(e["target"], 0xFFFFFFFF)
                pts = e.get("geom_points", [])
                geom_count = len(pts)
                if geom_count > 0:
                    geom_offset = len(geom_records)
                    for pt_lat, pt_lon in pts:
                        plat_scaled = int(round(pt_lat * 1e7))
                        plon_scaled = int(round(pt_lon * 1e7))
                        geom_records.extend(struct.pack("<ii", plat_scaled, plon_scaled))
                else:
                    geom_offset = 0

                edge_records.extend(struct.pack("<IHHIIHH",
                                                target_idx,
                                                e["cost"],
                                                e["length_m"],
                                                e["street_id"],
                                                geom_offset,
                                                geom_count,
                                                e.get("edge_flags", 0)))
                curr_edge_idx += 1

        with open(nodes_bin_path, "wb") as f:
            f.write(node_records)
        with open(edges_bin_path, "wb") as f:
            f.write(edge_records)
        with open(geom_bin_path, "wb") as f:
            f.write(geom_records)

    # Export global car backbone network
    write_graph_binary(backbone_adjacency,
                       os.path.join(OUTPUT_DIR, "backbone_nodes.bin"),
                       os.path.join(OUTPUT_DIR, "backbone_edges.bin"),
                       os.path.join(OUTPUT_DIR, "backbone_geom.bin"))

    # Export global stroller backbone network
    write_graph_binary(stroller_backbone_adjacency,
                       os.path.join(OUTPUT_DIR, "stroller_backbone_nodes.bin"),
                       os.path.join(OUTPUT_DIR, "stroller_backbone_edges.bin"),
                       os.path.join(OUTPUT_DIR, "stroller_backbone_geom.bin"))

    # Export spatially tiled local networks (with car/stroller/both edge flags)
    for grid_id, local_adj in local_adjacency.items():
        write_graph_binary(local_adj,
                           os.path.join(OUTPUT_DIR, f"local_{grid_id}_nodes.bin"),
                           os.path.join(OUTPUT_DIR, f"local_{grid_id}_edges.bin"),
                           os.path.join(OUTPUT_DIR, f"local_{grid_id}_geom.bin"))

    print(f"[Done] All binary datasets successfully written to '{OUTPUT_DIR}'.")


if __name__ == "__main__":
    run_pipeline()
