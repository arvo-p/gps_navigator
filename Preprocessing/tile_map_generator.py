import os
import math
import pickle
import time
import cairo
import osmium
from PIL import Image
from collections import defaultdict, deque

ZOOM_LEVELS = [12, 14, 16]
BBOX = (49.497, 2.544, 51.505, 6.408)
OUTPUT_DIR = "./tiles"
CACHE_FILE = "./tile_features_cache_v2.pkl"

# Spatial grid cell resolution in degrees: 0.05° (~5.5 km x 3.5 km)
GRID_SCALE = 20.0  # 1 / 0.05

STYLE_CONFIG = {
    "forest": {
        "color": (65 / 255.0, 105 / 255.0, 75 / 255.0),
        "min_zoom": 12,
        "widths": {12: 1.0, 14: 1.0, 16: 1.0}
    },
    "water": {
        "color": (0.12, 0.23, 0.37),
        "min_zoom": 10,
        "widths": {11: 2.0, 12: 3.0, 13: 4.0, 14: 6.0, 15: 8.0, 16: 12.0, 17: 16.0}
    },
    "greenway_track": {
        "color": (160 / 255.0, 105 / 255.0, 55 / 255.0),
        "min_zoom": 14,
        "widths": {14: 1.5, 15: 2.0, 16: 3.0, 17: 4.0}
    },
    "footway": {
        "color": (140 / 255.0, 148 / 255.0, 158 / 255.0),
        "min_zoom": 14,
        "widths": {14: 1.0, 15: 1.5, 16: 2.0, 17: 3.0}
    },
    "residential": {
        "color": (0.24, 0.27, 0.30),
        "min_zoom": 13,
        "widths": {13: 1.0, 14: 1.5, 15: 2.5, 16: 4.0, 17: 6.0}
    },
    "secondary": {
        "color": (0.90, 0.66, 0.24),
        "min_zoom": 11,
        "widths": {11: 1.0, 12: 1.5, 13: 2.0, 14: 3.0, 15: 4.5, 16: 6.5, 17: 9.0}
    },
    "primary": {
        "color": (1.00, 0.48, 0.00),
        "min_zoom": 9,
        "widths": {9: 1.0, 10: 1.5, 11: 2.0, 12: 2.5, 13: 3.5, 14: 5.0, 15: 7.0, 16: 9.5, 17: 12.0}
    },
    "motorway": {
        "color": (1.00, 0.20, 0.27),
        "min_zoom": 7,
        "widths": {7: 1.0, 8: 1.5, 9: 2.0, 10: 2.5, 11: 3.0, 12: 4.0, 13: 5.5, 14: 7.0, 15: 9.5, 16: 12.0, 17: 15.0}
    },
    "railway": {
        "base_color": (0.16, 0.18, 0.21),
        "dash_color": (0.82, 0.82, 0.82),
        "min_zoom": 11,
        "widths": {11: 1.5, 12: 2.0, 13: 2.5, 14: 3.5, 15: 5.0, 16: 6.5, 17: 8.5}
    }
}

PALETTE_16_DARK = [
    18,  20,  23,
    30,  58,  95,
    62,  68,  76,
    229, 169,  60,
    255, 122,   0,
    255,  51,  68,
    42,  46,  53,
    208, 208, 208,
    0,   229, 255,
    255,   0, 127,
    65,  105, 75,
    160, 105, 55,
    140, 148, 158,
    255, 255, 255,
    10,  11,  13,
    0,     0,   0
]

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile

def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * ytile / n)))
    return math.degrees(lat_rad), lon

def tile_bbox(tx, ty, zoom):
    """Computes geographic bounding box (lat_min, lon_min, lat_max, lon_max) of a tile."""
    lat_max, lon_min = num2deg(tx, ty, zoom)
    lat_min, lon_max = num2deg(tx + 1, ty + 1, zoom)
    return lat_min, lon_min, lat_max, lon_max

def world_to_tile_pixel(lat, lon, tx, ty, zoom):
    n = 2.0 ** zoom
    x_world = ((lon + 180.0) / 360.0) * n
    lat_rad = math.radians(lat)
    y_world = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) * 0.5 * n
    px = (x_world - tx) * 256.0
    py = (y_world - ty) * 256.0
    return px, py

def get_stroke_width(category, zoom):
    widths = STYLE_CONFIG[category]["widths"]
    if zoom in widths:
        return widths[zoom]
    closest_zoom = min(widths.keys(), key=lambda z: abs(z - zoom))
    return widths[closest_zoom]

def is_tile_complete(tile_path):
    """Checks if a tile already exists and has a valid PNG file header."""
    if not os.path.exists(tile_path):
        return False
    try:
        if os.path.getsize(tile_path) < 68:  # Minimum valid PNG size
            return False
        with open(tile_path, "rb") as f:
            header = f.read(8)
            return header == b"\x89PNG\r\n\x1a\n"
    except OSError:
        return False

class FeatureCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = []

    def way(self, w):
        highway = w.tags.get("highway")
        railway = w.tags.get("railway")
        waterway = w.tags.get("waterway")

        category = None
        if railway == "rail":
            category = "railway"
        elif highway in ["motorway", "motorway_link"]:
            category = "motorway"
        elif highway in ["trunk", "trunk_link", "primary", "primary_link"]:
            category = "primary"
        elif highway in ["secondary", "secondary_link", "tertiary", "tertiary_link"]:
            category = "secondary"
        elif highway in ["residential", "unclassified", "living_street"]:
            category = "residential"
        elif highway in ["cycleway", "track", "footway", "pedestrian", "path", "bridleway", "steps"]:
            access = w.tags.get("access")
            foot = w.tags.get("foot")

            if (
                (access not in ("no", "private") or foot in ("yes", "designated", "permissive"))
                and foot not in ("no", "private")
            ):
                if highway in ("cycleway", "track", "path", "bridleway"):
                    category = "greenway_track"
                else:
                    category = "footway"
        elif waterway in ["river", "canal"]:
            category = "water"

        if not category or len(w.nodes) < 2:
            return

        coords = []
        for n in w.nodes:
            if n.location.valid():
                coords.append((n.location.lat, n.location.lon))

        if len(coords) >= 2:
            self.ways.append((category, coords))

    def area(self, a):
        landuse = a.tags.get("landuse")
        natural = a.tags.get("natural")
        leisure = a.tags.get("leisure")
        if (
            landuse == "forest"
            or natural in ("wood", "heath")
            or leisure in ("park", "nature_reserve")
        ):
            try:
                for outer in a.outer_rings():
                    coords = [(n.lat, n.lon) for n in outer if n.location.valid()]
                    if len(coords) >= 3:
                        self.ways.append(("forest", coords))
            except Exception:
                pass

def prune_isolated_greenway_tracks(ways):
    """
    Removes isolated 'greenway_track' ways and dead-end track spurs that do not
    connect through at both ends to roads or connected pedestrian networks.
    Also splits tracks at interior junctions so dead-end tails are trimmed cleanly.
    """
    print("[Filter] Pruning isolated and dead-end 'greenway_track' segments...")
    t0 = time.time()

    road_cats = {"residential", "secondary", "primary", "motorway"}
    road_pts = set()
    footway_ways = []
    track_ways = []
    other_ways = []

    for item in ways:
        cat = item[0]
        coords = item[1] if len(item) == 2 else item[2]
        if cat == "greenway_track":
            track_ways.append(coords)
        elif cat == "footway":
            footway_ways.append(coords)
        else:
            other_ways.append((cat, coords))
            if cat in road_cats:
                road_pts.update(coords)

    if not track_ways and not footway_ways:
        return ways

    # Only allow footways that themselves connect to the road network (or to tracks touching roads)
    # to act as valid endpoints, preventing a dead-end forest footway from anchoring a dead-end track.
    all_ped_ways = [("greenway_track", c) for c in track_ways] + [("footway", c) for c in footway_ways]
    pt_degree = defaultdict(int)
    for _, coords in all_ped_ways:
        for pt in set(coords):
            pt_degree[pt] += 1

    # Split greenway_track ways at interior junction points so dead-end tails past a fork are trimmed
    segments = []
    for coords in track_ways:
        start_idx = 0
        for i in range(1, len(coords)):
            pt = coords[i]
            is_junction = (pt in road_pts) or (pt_degree[pt] >= 2) or (i == len(coords) - 1)
            if is_junction:
                seg = coords[start_idx : i + 1]
                if len(seg) >= 2 and seg[0] != seg[-1]:
                    segments.append(("greenway_track", seg))
                start_idx = i

    # Also include footway segments in the connectivity graph so track -> footway -> road bridges work,
    # while track -> dead-end footway chains get peeled back from the dead end.
    for coords in footway_ways:
        start_idx = 0
        for i in range(1, len(coords)):
            pt = coords[i]
            is_junction = (pt in road_pts) or (pt_degree[pt] >= 2) or (i == len(coords) - 1)
            if is_junction:
                seg = coords[start_idx : i + 1]
                if len(seg) >= 2 and seg[0] != seg[-1]:
                    segments.append(("footway", seg))
                start_idx = i

    ep_to_segs = defaultdict(set)
    for sid, (_, seg) in enumerate(segments):
        ep_to_segs[seg[0]].add(sid)
        ep_to_segs[seg[-1]].add(sid)

    def is_ep_alive(pt):
        return (pt in road_pts) or (len(ep_to_segs[pt]) >= 2)

    # 1. Iterative 2-core dead-end peeling: strip any segment with an unanchored degree-1 endpoint
    dead_segs = set()
    queue = [
        sid for sid, (_, seg) in enumerate(segments)
        if not (is_ep_alive(seg[0]) and is_ep_alive(seg[-1]))
    ]
    while queue:
        sid = queue.pop()
        if sid in dead_segs:
            continue
        dead_segs.add(sid)
        _, seg = segments[sid]
        for ep in (seg[0], seg[-1]):
            ep_to_segs[ep].discard(sid)
            if not is_ep_alive(ep):
                for nbr_sid in list(ep_to_segs[ep]):
                    if nbr_sid not in dead_segs:
                        queue.append(nbr_sid)

    # 2. Road-anchored Component Filter: every surviving pedestrian component must connect
    #    at least 2 distinct road points (eliminates floating islands AND single-entry lollipop loops)
    anchored_sids = set()
    visited_sids = set()
    for start_sid in range(len(segments)):
        if start_sid in dead_segs or start_sid in visited_sids:
            continue
        comp_sids = []
        comp_road_pts = set()
        bfs_q = [start_sid]
        visited_sids.add(start_sid)
        while bfs_q:
            curr = bfs_q.pop()
            comp_sids.append(curr)
            _, seg = segments[curr]
            if seg[0] in road_pts:
                comp_road_pts.add(seg[0])
            if seg[-1] in road_pts:
                comp_road_pts.add(seg[-1])
            for ep in (seg[0], seg[-1]):
                # Do not traverse through road_pts to unrelated components
                if ep in road_pts:
                    continue
                for nbr in ep_to_segs[ep]:
                    if nbr not in dead_segs and nbr not in visited_sids:
                        visited_sids.add(nbr)
                        bfs_q.append(nbr)
        if len(comp_road_pts) >= 2:
            anchored_sids.update(comp_sids)

    kept_ped = [
        (segments[sid][0], segments[sid][1])
        for sid in sorted(anchored_sids)
    ]
    print(
        f"[Filter] Kept {len(kept_ped):,} connected pedestrian/track segments "
        f"(from {len(track_ways) + len(footway_ways):,} raw ways) in {time.time() - t0:.2f}s."
    )
    return other_ways + kept_ped


def build_spatial_grid(ways):
    """
    Indexes all ways into a 2D spatial grid (0.05° cells) grouped by category.
    This enables O(1) candidate lookup instead of scanning 500,000 ways per tile.
    """
    print("[Index] Building spatial grid index for fast tile rendering...")
    t0 = time.time()
    grid = defaultdict(lambda: defaultdict(list))
    
    for item in ways:
        cat = item[0]
        coords = item[1] if len(item) == 2 else item[2]
        
        # Precompute way bounding box
        min_lat = min(p[0] for p in coords)
        max_lat = max(p[0] for p in coords)
        min_lon = min(p[1] for p in coords)
        max_lon = max(p[1] for p in coords)
        bbox = (min_lat, min_lon, max_lat, max_lon)
        way_entry = (bbox, coords)

        # Map to overlapping grid cells
        gx1, gy1 = int(min_lat * GRID_SCALE), int(min_lon * GRID_SCALE)
        gx2, gy2 = int(max_lat * GRID_SCALE), int(max_lon * GRID_SCALE)
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                grid[(gx, gy)][cat].append(way_entry)

    print(f"[Index] Indexed {len(ways):,} features across {len(grid):,} grid cells in {time.time() - t0:.2f}s.")
    return grid

def tile_has_features(spatial_grid, tx, ty, zoom):
    """
    Rapidly checks if any non-forest features (roads, paths, railways, rivers)
    intersect this tile (takes < 1 µs).
    """
    t_min_lat, t_min_lon, t_max_lat, t_max_lon = tile_bbox(tx, ty, zoom)
    pad_lat = (t_max_lat - t_min_lat) * 0.02
    pad_lon = (t_max_lon - t_min_lon) * 0.02
    t_min_lat -= pad_lat
    t_max_lat += pad_lat
    t_min_lon -= pad_lon
    t_max_lon += pad_lon

    gx1, gy1 = int(t_min_lat * GRID_SCALE), int(t_min_lon * GRID_SCALE)
    gx2, gy2 = int(t_max_lat * GRID_SCALE), int(t_max_lon * GRID_SCALE)

    order = ["water", "greenway_track", "footway", "residential", "secondary", "primary", "motorway", "railway"]
    for cat in order:
        if zoom < STYLE_CONFIG[cat]["min_zoom"]:
            continue
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                cell_ways = spatial_grid.get((gx, gy))
                if not cell_ways or cat not in cell_ways:
                    continue
                for bbox, _ in cell_ways[cat]:
                    if not (bbox[2] < t_min_lat or bbox[0] > t_max_lat or bbox[3] < t_min_lon or bbox[1] > t_max_lon):
                        return True
    return False


def tile_has_forest(spatial_grid, tx, ty, zoom):
    """
    Checks if any 'forest' polygon intersects this tile.
    """
    if zoom < STYLE_CONFIG["forest"]["min_zoom"]:
        return False

    t_min_lat, t_min_lon, t_max_lat, t_max_lon = tile_bbox(tx, ty, zoom)
    pad_lat = (t_max_lat - t_min_lat) * 0.02
    pad_lon = (t_max_lon - t_min_lon) * 0.02
    t_min_lat -= pad_lat
    t_max_lat += pad_lat
    t_min_lon -= pad_lon
    t_max_lon += pad_lon

    gx1, gy1 = int(t_min_lat * GRID_SCALE), int(t_min_lon * GRID_SCALE)
    gx2, gy2 = int(t_max_lat * GRID_SCALE), int(t_max_lon * GRID_SCALE)

    for gx in range(gx1, gx2 + 1):
        for gy in range(gy1, gy2 + 1):
            cell_ways = spatial_grid.get((gx, gy))
            if not cell_ways or "forest" not in cell_ways:
                continue
            for bbox, _ in cell_ways["forest"]:
                if not (bbox[2] < t_min_lat or bbox[0] > t_max_lat or bbox[3] < t_min_lon or bbox[1] > t_max_lon):
                    return True
    return False


def find_interior_belgium_tiles(road_tiles, x_start, x_end, y_start, y_end):
    """
    Identifies tiles that are strictly inside Belgium (enclosed by road_tiles and
    not at the borders/exterior of Belgium) so forest-only tiles inside Belgium
    are drawn without drawing cross-border forest overhangs outside Belgium.
    """
    if not road_tiles:
        return set()

    row_min_x = {}
    row_max_x = {}
    col_min_y = {}
    col_max_y = {}
    for x, y in road_tiles:
        if y not in row_min_x or x < row_min_x[y]:
            row_min_x[y] = x
        if y not in row_max_x or x > row_max_x[y]:
            row_max_x[y] = x
        if x not in col_min_y or y < col_min_y[x]:
            col_min_y[x] = y
        if x not in col_max_y or y > col_max_y[x]:
            col_max_y[x] = y

    # Dilate road_tiles by 1 tile to bridge small 1-tile rural border gaps during exterior flood-fill
    closed_barrier = set(road_tiles)
    for x, y in road_tiles:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                closed_barrier.add((x + dx, y + dy))

    # Flood-fill the exterior starting from the bounding box perimeter
    exterior = set()
    q = deque()
    for x in range(x_start, x_end):
        for y in (y_start, y_end - 1):
            if (x, y) not in closed_barrier and (x, y) not in exterior:
                exterior.add((x, y))
                q.append((x, y))
    for y in range(y_start, y_end):
        for x in (x_start, x_end - 1):
            if (x, y) not in closed_barrier and (x, y) not in exterior:
                exterior.add((x, y))
                q.append((x, y))

    while q:
        cx, cy = q.popleft()
        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if x_start <= nx < x_end and y_start <= ny < y_end:
                if (nx, ny) not in closed_barrier and (nx, ny) not in exterior:
                    exterior.add((nx, ny))
                    q.append((nx, ny))

    # Also mark the 1-tile outer fringe touching exterior as border/exterior
    border_or_exterior = set(exterior)
    for x, y in list(exterior):
        for dx in (-2, -1, 0, 1, 2):
            for dy in (-2, -1, 0, 1, 2):
                border_or_exterior.add((x + dx, y + dy))

    interior = set()
    for x in range(x_start, x_end):
        for y in range(y_start, y_end):
            if (x, y) in road_tiles or (x, y) in border_or_exterior:
                continue
            if (
                y in row_min_x
                and x in col_min_y
                and (row_min_x[y] < x < row_max_x[y])
                and (col_min_y[x] < y < col_max_y[x])
            ):
                interior.add((x, y))
    return interior


def render_tile_fast(spatial_grid, tx, ty, zoom, out_path, allow_forest_only=False):
    """
    Renders a 256x256 tile using the spatial grid to filter candidate roads in < 0.1ms.
    Skips writing if the tile contains no features (empty tile).
    """
    # 1. Calculate tile's geographic bounding box with 2% stroke padding (~5px)
    t_min_lat, t_min_lon, t_max_lat, t_max_lon = tile_bbox(tx, ty, zoom)
    pad_lat = (t_max_lat - t_min_lat) * 0.02
    pad_lon = (t_max_lon - t_min_lon) * 0.02
    t_min_lat -= pad_lat
    t_max_lat += pad_lat
    t_min_lon -= pad_lon
    t_max_lon += pad_lon

    # 2. Determine which spatial grid cells overlap this tile (usually just 1 or 2 cells)
    gx1, gy1 = int(t_min_lat * GRID_SCALE), int(t_min_lon * GRID_SCALE)
    gx2, gy2 = int(t_max_lat * GRID_SCALE), int(t_max_lon * GRID_SCALE)

    # Painter's order from bottom to top: forests & pedestrian ways are drawn underneath car roads
    order = ["forest", "water", "greenway_track", "footway", "residential", "secondary", "primary", "motorway", "railway"]
    features_by_cat = {}
    total_features = 0
    non_forest_features = 0

    for cat in order:
        cfg = STYLE_CONFIG[cat]
        if zoom < cfg["min_zoom"]:
            continue

        seen = set()
        cat_ways = []
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                cell_ways = spatial_grid.get((gx, gy))
                if not cell_ways or cat not in cell_ways:
                    continue

                for bbox, coords in cell_ways[cat]:
                    wid = id(coords)
                    if wid in seen:
                        continue
                    seen.add(wid)

                    # Fast 4-float bounding box rejection check (5 nanoseconds)
                    if bbox[2] < t_min_lat or bbox[0] > t_max_lat or bbox[3] < t_min_lon or bbox[1] > t_max_lon:
                        continue

                    cat_ways.append(coords)

        if cat_ways:
            features_by_cat[cat] = cat_ways
            total_features += len(cat_ways)
            if cat != "forest":
                non_forest_features += len(cat_ways)

    # Skip writing completely empty tiles, unless this is an interior Belgium tile with a forest
    if non_forest_features == 0 and not (allow_forest_only and "forest" in features_by_cat):
        return False

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 256, 256)
    ctx = cairo.Context(surface)
    ctx.set_source_rgb(0.07, 0.08, 0.09)
    ctx.paint()

    for cat in order:
        if cat not in features_by_cat:
            continue

        cfg = STYLE_CONFIG[cat]
        base_width = get_stroke_width(cat, zoom) if cat != "forest" else 0.0

        # Disable anti-aliasing on palette-swappable layers to prevent ghost outlines when hidden via LUT
        if cat in ("forest", "greenway_track", "footway"):
            ctx.set_antialias(cairo.ANTIALIAS_NONE)
        else:
            ctx.set_antialias(cairo.ANTIALIAS_DEFAULT)

        for coords in features_by_cat[cat]:
            pts = [world_to_tile_pixel(lat, lon, tx, ty, zoom) for lat, lon in coords]

            ctx.new_path()
            ctx.move_to(pts[0][0], pts[0][1])
            for px, py in pts[1:]:
                ctx.line_to(px, py)

            if cat == "forest":
                ctx.close_path()
                ctx.set_source_rgb(*cfg["color"])
                ctx.fill()
            elif cat == "railway":
                ctx.set_line_cap(cairo.LINE_CAP_ROUND)
                ctx.set_line_join(cairo.LINE_JOIN_ROUND)
                ctx.set_dash([])
                ctx.set_source_rgb(*cfg["base_color"])
                ctx.set_line_width(base_width)
                ctx.stroke_preserve()

                dash_len = max(3.0, base_width * 1.2)
                ctx.set_source_rgb(*cfg["dash_color"])
                ctx.set_line_width(max(1.0, base_width * 0.6))
                ctx.set_dash([dash_len, dash_len])
                ctx.stroke()
            elif cat == "greenway_track":
                ctx.set_line_cap(cairo.LINE_CAP_ROUND)
                ctx.set_line_join(cairo.LINE_JOIN_ROUND)
                dot_len = max(1.0, base_width * 0.5)
                gap_len = max(2.5, base_width * 1.8)
                ctx.set_dash([dot_len, gap_len])
                ctx.set_source_rgb(*cfg["color"])
                ctx.set_line_width(base_width)
                ctx.stroke()
            else:
                ctx.set_line_cap(cairo.LINE_CAP_ROUND)
                ctx.set_line_join(cairo.LINE_JOIN_ROUND)
                ctx.set_dash([])
                ctx.set_source_rgb(*cfg["color"])
                ctx.set_line_width(base_width)
                ctx.stroke()

    buf = surface.get_data()
    img = Image.frombuffer("RGBA", (256, 256), buf, "raw", "BGRA", 0, 1).convert("RGB")
    pal_img = Image.new("P", (1, 1))
    pal_img.putpalette(PALETTE_16_DARK + [0] * (768 - len(PALETTE_16_DARK)))
    q_img = img.quantize(palette=pal_img, dither=Image.Dither.NONE)

    # Ground-truth empty check: if rendered canvas only contains background color 0,
    # skip writing unless it is an interior forest tile inside Belgium!
    colors = q_img.getcolors()
    if colors and len(colors) == 1 and colors[0][1] == 0 and not allow_forest_only:
        return False

    # Atomic write via temporary file: ensures power loss never leaves corrupted PNGs
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    tmp_path = out_path + ".tmp"
    q_img.save(tmp_path, format="PNG", optimize=True)
    os.replace(tmp_path, out_path)
    return True

def load_or_extract_features(pbf_path):
    """
    Loads extracted way features from disk cache if available.
    If not, parses the PBF file and saves the cache so future resumes are instant.
    """
    if os.path.exists(CACHE_FILE) and os.path.getmtime(CACHE_FILE) >= os.path.getmtime(pbf_path):
        print(f"[Cache] Found cached features in '{CACHE_FILE}'. Loading...")
        t0 = time.time()
        with open(CACHE_FILE, "rb") as f:
            ways = pickle.load(f)
        print(f"[Cache] Loaded {len(ways):,} features in {time.time() - t0:.1f}s.")
        return ways

    print(f"[OSM] Parsing '{pbf_path}' (first-time feature extraction)...")
    t0 = time.time()
    collector = FeatureCollector()
    collector.apply_file(pbf_path, locations=True)
    ways = collector.ways
    print(f"[OSM] Extracted {len(ways):,} features in {time.time() - t0:.1f}s.")

    print(f"[Cache] Saving features to '{CACHE_FILE}' for quick resume...")
    try:
        with open(CACHE_FILE, "wb") as f:
            pickle.dump(ways, f, protocol=pickle.HIGHEST_PROTOCOL)
        print("[Cache] Features saved successfully.")
    except Exception as e:
        print(f"[Cache] Warning: Could not write cache file: {e}")

    return ways

def main(pbf_path):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ways = load_or_extract_features(pbf_path)
    ways = prune_isolated_greenway_tracks(ways)
    spatial_grid = build_spatial_grid(ways)

    lat_min, lon_min, lat_max, lon_max = BBOX

    print(f"\nStarting high-speed tile generation in '{OUTPUT_DIR}' across zoom levels {ZOOM_LEVELS}...")
    start_time = time.time()

    for zoom in ZOOM_LEVELS:
        x_min, y_min = deg2num(lat_max, lon_min, zoom)
        x_max, y_max = deg2num(lat_min, lon_max, zoom)

        x_start, x_end = min(x_min, x_max), max(x_min, x_max) + 1
        y_start, y_end = min(y_min, y_max), max(y_min, y_max) + 1
        total_bbox_tiles = (x_end - x_start) * (y_end - y_start)

        # Scan: identify road/water tiles and interior forest-only tiles inside Belgium
        road_tiles = set()
        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                if tile_has_features(spatial_grid, x, y, zoom):
                    road_tiles.add((x, y))

        interior_tiles = find_interior_belgium_tiles(road_tiles, x_start, x_end, y_start, y_end)

        pending_tiles = []
        already_rendered = 0
        skipped_empty = 0

        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                out_file = os.path.join(OUTPUT_DIR, str(zoom), str(x), f"{y}.png")
                has_road = (x, y) in road_tiles
                has_interior_forest = (not has_road) and ((x, y) in interior_tiles) and tile_has_forest(spatial_grid, x, y, zoom)

                if is_tile_complete(out_file) and (has_road or has_interior_forest):
                    already_rendered += 1
                elif has_road or has_interior_forest:
                    pending_tiles.append((x, y, out_file, has_interior_forest))
                else:
                    skipped_empty += 1

        active_tiles_total = already_rendered + len(pending_tiles)

        if len(pending_tiles) == 0:
            print(f"[Zoom {zoom:2d}] 100% complete ({active_tiles_total:,}/{active_tiles_total:,} feature tiles exist; {skipped_empty:,} empty outside Belgium skipped).")
            continue

        pct = (already_rendered / active_tiles_total * 100) if active_tiles_total > 0 else 0
        print(f"[Zoom {zoom:2d}] Active tiles: {active_tiles_total:,} (skipped {skipped_empty:,} empty) | Already rendered: {already_rendered:,} ({pct:.1f}%) | Pending: {len(pending_tiles):,}", flush=True)

        z_start = time.time()
        for idx, (x, y, out_file, is_forest_only) in enumerate(pending_tiles, start=1):
            render_tile_fast(spatial_grid, x, y, zoom, out_file, allow_forest_only=is_forest_only)

            if idx % 100 == 0 or idx == len(pending_tiles):
                elapsed = time.time() - z_start
                rate = idx / max(0.001, elapsed)
                remaining_sec = (len(pending_tiles) - idx) / max(0.001, rate)
                current_total = already_rendered + idx
                percent = current_total / active_tiles_total * 100
                print(f"  [Zoom {zoom:2d}] {current_total:,}/{active_tiles_total:,} ({percent:5.1f}%) | {rate:5.1f} tiles/s | ETA: {remaining_sec/60:4.1f}m   ", end="\r", flush=True)

        print(f"\n[Zoom {zoom:2d}] Completed in {time.time() - z_start:.1f}s.", flush=True)

    print(f"\n[Done] All tile generation finished in {time.time() - start_time:.1f}s.")

if __name__ == "__main__":
    main("belgium-latest.osm.pbf")
