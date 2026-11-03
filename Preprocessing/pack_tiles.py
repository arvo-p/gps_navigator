"""
convert_tiles_to_bmp.py
=======================
Packs PNG map tiles in ./tiles/{z}/{x}/{y}.png into contiguous, 512-byte
sector-aligned archive files (tiles_z{z}.pak) for fast O(1) FatFs SD card reads
on STM32 (using _USE_FASTSEEK and multi-block CMD18 DMA transfers).

Binary Layout of tiles_z{z}.pak:
┌──────────────────────────────────────────────────────────────────────────┐
│ Header (24 bytes, '<4s5I'):                                              │
│   magic ("TPAK"), zoom, x_min, x_max, y_min, y_max                       │
├──────────────────────────────────────────────────────────────────────────┤
│ Offset Table [num_x * num_y] of uint32_t ('<I'):                         │
│   Indexed by: grid_idx = (x - x_min) * num_y + (y - y_min)               │
│   Value: 512-byte sector index of tile (0 = empty/missing tile)          │
│   Byte offset in file = (FSIZE_t)sector_idx * 512                        │
├──────────────────────────────────────────────────────────────────────────┤
│ Padding to 512-byte sector boundary                                      │
├──────────────────────────────────────────────────────────────────────────┤
│ Shared Global Palette (3 sectors = 1,536 bytes):                         │
│   - Sector +0 (512 B):  256 × uint16_t RGB565 LUT (for ST7789 / SPI LCD) │
│   - Sector +1..2 (1024 B): 256 × 4-byte BGRA8888 CLUT (for STM32 DMA2D)  │
├──────────────────────────────────────────────────────────────────────────┤
│ Contiguous Tile Data (ordered by (x, y)):                                │
│   - Each existing tile is 65,536 bytes (256×256 8-bit indexed pixels)    │
│   - Exactly 128 contiguous 512-byte sectors per tile                     │
└──────────────────────────────────────────────────────────────────────────┘

Also supports legacy individual 8-bit indexed BMP output via --legacy-bmp.
"""

import os
import sys
import time
import zlib
import struct
import argparse
from concurrent.futures import ProcessPoolExecutor

DEFAULT_INPUT_DIR = "./tiles"
DEFAULT_OUTPUT_DIR = "./tiles_bmp"

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
PAK_MAGIC = b"TPAK"
PAK_HEADER_FMT = "<4s5I"
PAK_HEADER_SIZE = 24  # struct.calcsize("<4s5I")
SECTOR_SIZE = 512
TILE_WIDTH = 256
TILE_HEIGHT = 256
TILE_BYTES = TILE_WIDTH * TILE_HEIGHT  # 65,536 bytes (128 sectors of 512B)
SECTORS_PER_TILE = TILE_BYTES // SECTOR_SIZE  # 128


def is_bmp_complete(bmp_path):
    """Checks if a target BMP file already exists and has a valid BMP header and size."""
    if not os.path.exists(bmp_path):
        return False
    try:
        # 256x256 8-bit indexed BMP is 54 (headers) + 1024 (palette) + 65536 (pixels) = 66,614 bytes
        if os.path.getsize(bmp_path) < 66614:
            return False
        with open(bmp_path, "rb") as f:
            return f.read(2) == b"BM"
    except OSError:
        return False


def is_pak_complete(pak_path, expected_zoom, x_min, x_max, y_min, y_max, num_existing_tiles):
    """Checks if a target .pak file already exists with valid header and exact expected byte size."""
    if not os.path.exists(pak_path):
        return False
    try:
        num_x = x_max - x_min + 1
        num_y = y_max - y_min + 1
        table_end = PAK_HEADER_SIZE + (num_x * num_y * 4)
        palette_sector = (table_end + SECTOR_SIZE - 1) // SECTOR_SIZE
        first_tile_sector = palette_sector + 3  # 1 sector RGB565 + 2 sectors BGRA8888
        expected_size = first_tile_sector * SECTOR_SIZE + num_existing_tiles * TILE_BYTES

        if os.path.getsize(pak_path) != expected_size:
            return False

        with open(pak_path, "rb") as f:
            hdr = f.read(PAK_HEADER_SIZE)
            if len(hdr) != PAK_HEADER_SIZE:
                return False
            magic, zoom, hx_min, hx_max, hy_min, hy_max = struct.unpack(PAK_HEADER_FMT, hdr)
            return (
                magic == PAK_MAGIC
                and zoom == expected_zoom
                and hx_min == x_min
                and hx_max == x_max
                and hy_min == y_min
                and hy_max == y_max
            )
    except OSError:
        return False


def _unfilter_scanlines(raw_data, width, height, bpp=1):
    """Reconstructs PNG scanlines supporting all 5 standard PNG filter types (0..4)."""
    row_bytes = width * bpp
    stride = row_bytes + 1
    out = bytearray(height * row_bytes)
    prev_row = bytearray(row_bytes)

    for y in range(height):
        row_start = y * stride
        filt = raw_data[row_start]
        src = raw_data[row_start + 1 : row_start + stride]
        dst_offset = y * row_bytes

        if filt == 0:  # None
            out[dst_offset : dst_offset + row_bytes] = src
        elif filt == 1:  # Sub
            cur = bytearray(src)
            for i in range(bpp, row_bytes):
                cur[i] = (cur[i] + cur[i - bpp]) & 0xFF
            out[dst_offset : dst_offset + row_bytes] = cur
        elif filt == 2:  # Up
            cur = bytearray(
                (src[i] + prev_row[i]) & 0xFF for i in range(row_bytes)
            )
            out[dst_offset : dst_offset + row_bytes] = cur
        elif filt == 3:  # Average
            cur = bytearray(src)
            for i in range(row_bytes):
                left = cur[i - bpp] if i >= bpp else 0
                up = prev_row[i]
                cur[i] = (cur[i] + ((left + up) >> 1)) & 0xFF
            out[dst_offset : dst_offset + row_bytes] = cur
        elif filt == 4:  # Paeth
            cur = bytearray(src)
            for i in range(row_bytes):
                a = cur[i - bpp] if i >= bpp else 0
                b = prev_row[i]
                c = prev_row[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa = abs(p - a)
                pb = abs(p - b)
                pc = abs(p - c)
                if pa <= pb and pa <= pc:
                    pr = a
                elif pb <= pc:
                    pr = b
                else:
                    pr = c
                cur[i] = (cur[i] + pr) & 0xFF
            out[dst_offset : dst_offset + row_bytes] = cur
        else:
            raise ValueError(f"Unsupported PNG filter type: {filt}")

        prev_row = out[dst_offset : dst_offset + row_bytes]

    return out


def decode_indexed_png(png_bytes):
    """
    Decodes an 8-bit indexed PNG byte buffer and returns:
      (width, height, plte_data, pixels_top_down)
    where pixels_top_down is a bytearray of length width * height (65,536 bytes for 256x256).
    """
    if png_bytes[:8] != PNG_SIGNATURE:
        raise ValueError("Invalid PNG signature")

    pos = 8
    width = height = bit_depth = color_type = interlace = None
    plte_data = b""
    idat_chunks = []

    data_len = len(png_bytes)
    while pos < data_len:
        length, ctype = struct.unpack_from(">I4s", png_bytes, pos)
        chunk_start = pos + 8
        chunk_end = chunk_start + length
        pos = chunk_end + 4  # Skip 4-byte CRC

        if ctype == b"IHDR":
            width, height, bit_depth, color_type, _, _, interlace = struct.unpack_from(
                ">IIBBBBB", png_bytes, chunk_start
            )
        elif ctype == b"PLTE":
            plte_data = png_bytes[chunk_start:chunk_end]
        elif ctype == b"IDAT":
            idat_chunks.append(png_bytes[chunk_start:chunk_end])
        elif ctype == b"IEND":
            break

    if width is None or color_type != 3 or bit_depth != 8 or interlace != 0:
        raise ValueError(
            f"Expected 8-bit non-interlaced indexed PNG (got bit_depth={bit_depth}, "
            f"color_type={color_type}, interlace={interlace})"
        )

    raw_scanlines = zlib.decompress(b"".join(idat_chunks))
    pixels_top_down = _unfilter_scanlines(raw_scanlines, width, height, bpp=1)
    return width, height, plte_data, pixels_top_down


def build_palettes_from_plte(plte_data):
    """
    Converts PNG RGB PLTE data (up to 256 entries, 3 bytes each) into:
      - rgb565_lut: 512 bytes (256 × uint16_t little-endian RGB565)
      - bgra_lut:   1024 bytes (256 × 4-byte BGRA8888)
    """
    rgb565_lut = bytearray(512)
    bgra_lut = bytearray(1024)
    num_colors = min(256, len(plte_data) // 3)

    for i in range(num_colors):
        r = plte_data[i * 3]
        g = plte_data[i * 3 + 1]
        b = plte_data[i * 3 + 2]

        # BGRA8888 (for BMP / STM32 DMA2D CLUT)
        bgra_lut[i * 4] = b
        bgra_lut[i * 4 + 1] = g
        bgra_lut[i * 4 + 2] = r
        bgra_lut[i * 4 + 3] = 0

        # RGB565 (for ST7789 LCD)
        rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        struct.pack_into("<H", rgb565_lut, i * 2, rgb565)

    return bytes(rgb565_lut), bytes(bgra_lut)


def png_to_indexed_bmp_bytes(png_bytes):
    """
    Decodes an 8-bit indexed PNG byte buffer and returns a complete 8-bit indexed BMP byte buffer.
    """
    width, height, plte_data, pixels_top_down = decode_indexed_png(png_bytes)
    _, bmp_palette = build_palettes_from_plte(plte_data)

    # BMP scanlines are padded to 4-byte boundaries and stored bottom-up
    bmp_row_stride = (width + 3) & ~3
    pad_bytes = b"\x00" * (bmp_row_stride - width)
    image_size = bmp_row_stride * height
    pixel_offset = 14 + 40 + 1024
    file_size = pixel_offset + image_size

    bmp_header = struct.pack(
        "<2sIHHI",
        b"BM",
        file_size,
        0,
        0,
        pixel_offset,
    )
    dib_header = struct.pack(
        "<IiiHHIIiiII",
        40,              # biSize
        width,           # biWidth
        height,          # biHeight (positive = bottom-up)
        1,               # biPlanes
        8,               # biBitCount (8-bit indexed)
        0,               # biCompression (BI_RGB = uncompressed)
        image_size,      # biSizeImage
        2835,            # biXPelsPerMeter (72 DPI)
        2835,            # biYPelsPerMeter (72 DPI)
        256,             # biClrUsed
        256,             # biClrImportant
    )

    bmp_pixels = bytearray(image_size)
    mv_src = memoryview(pixels_top_down)
    mv_dst = memoryview(bmp_pixels)

    for y in range(height):
        src_start = (height - 1 - y) * width
        dst_start = y * bmp_row_stride
        mv_dst[dst_start : dst_start + width] = mv_src[src_start : src_start + width]
        if pad_bytes:
            mv_dst[dst_start + width : dst_start + bmp_row_stride] = pad_bytes

    return bmp_header + dib_header + bmp_palette + bmp_pixels


def decode_tile_for_pak(src_path):
    """Worker function to decode a single PNG tile into raw 65,536-byte 8-bit indexed pixels."""
    try:
        with open(src_path, "rb") as f:
            png_bytes = f.read()
        width, height, _, pixels_top_down = decode_indexed_png(png_bytes)
        if width != TILE_WIDTH or height != TILE_HEIGHT:
            raise ValueError(f"Unexpected tile dimensions {width}x{height} (expected 256x256)")
        return True, bytes(pixels_top_down)
    except Exception as exc:
        return False, f"{src_path}: {exc}"


def convert_single_tile(task):
    """Worker function to convert a single PNG tile to an 8-bit indexed BMP."""
    src_path, dst_path = task
    try:
        with open(src_path, "rb") as f:
            png_bytes = f.read()

        bmp_bytes = png_to_indexed_bmp_bytes(png_bytes)

        os.makedirs(os.path.dirname(dst_path), exist_ok=True)
        tmp_path = f"{dst_path}.{os.getpid()}.tmp"
        with open(tmp_path, "wb") as f:
            f.write(bmp_bytes)
        os.replace(tmp_path, dst_path)
        return True, None
    except Exception as exc:
        return False, f"{src_path}: {exc}"


def scan_zoom_levels(input_dir):
    """
    Scans input_dir/{zoom}/{x}/{y}.png and returns a sorted dict:
      zoom (int) -> list of (x, y, src_path) sorted by (x, y).
    """
    zoom_map = {}
    for entry in os.scandir(input_dir):
        if not entry.is_dir() or not entry.name.isdigit():
            continue
        zoom = int(entry.name)
        tiles = []
        for x_entry in os.scandir(entry.path):
            if not x_entry.is_dir() or not x_entry.name.isdigit():
                continue
            x = int(x_entry.name)
            for y_entry in os.scandir(x_entry.path):
                if y_entry.is_file() and y_entry.name.lower().endswith(".png"):
                    y_str = os.path.splitext(y_entry.name)[0]
                    if y_str.isdigit():
                        tiles.append((x, int(y_str), y_entry.path))
        if tiles:
            tiles.sort(key=lambda item: (item[0], item[1]))
            zoom_map[zoom] = tiles
    return dict(sorted(zoom_map.items()))


def build_zoom_pak(zoom, tiles, output_dir, workers, overwrite=False):
    """
    Packs all tiles for a single zoom level into output_dir/tiles_z{zoom}.pak.
    """
    pak_path = os.path.join(output_dir, f"tiles_z{zoom}.pak")

    xs = [t[0] for t in tiles]
    ys = [t[1] for t in tiles]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    num_x = x_max - x_min + 1
    num_y = y_max - y_min + 1
    grid_size = num_x * num_y

    if not overwrite and is_pak_complete(pak_path, zoom, x_min, x_max, y_min, y_max, len(tiles)):
        print(
            f"[Skip] tiles_z{zoom}.pak already complete "
            f"({len(tiles):,} tiles, grid {num_x}x{num_y})."
        )
        return 0, 0

    # Extract shared palette from the first tile
    with open(tiles[0][2], "rb") as f:
        _, _, plte_data, _ = decode_indexed_png(f.read())
    rgb565_lut, bgra_lut = build_palettes_from_plte(plte_data)

    # Calculate sector-aligned offsets
    table_bytes = grid_size * 4
    table_end = PAK_HEADER_SIZE + table_bytes
    palette_sector = (table_end + SECTOR_SIZE - 1) // SECTOR_SIZE
    first_tile_sector = palette_sector + 3  # 1 sector RGB565 (512B) + 2 sectors BGRA8888 (1024B)

    # Build Offset Table [num_x * num_y] of uint32_t 512B sector indices (0 = missing/empty)
    offset_table = [0] * grid_size
    for slot_idx, (x, y, _) in enumerate(tiles):
        col = x - x_min
        row = y - y_min
        grid_idx = col * num_y + row
        offset_table[grid_idx] = first_tile_sector + slot_idx * SECTORS_PER_TILE

    header_bytes = struct.pack(
        PAK_HEADER_FMT,
        PAK_MAGIC,
        zoom,
        x_min,
        x_max,
        y_min,
        y_max,
    )
    offset_table_bytes = struct.pack(f"<{grid_size}I", *offset_table)
    pad_len = (palette_sector * SECTOR_SIZE) - table_end
    padding_bytes = b"\x00" * pad_len

    os.makedirs(output_dir, exist_ok=True)
    tmp_path = f"{pak_path}.{os.getpid()}.tmp"

    total_tiles = len(tiles)
    expected_mb = (first_tile_sector * SECTOR_SIZE + total_tiles * TILE_BYTES) / (1024 * 1024)
    print(
        f"[Pack z{zoom}] Packing {total_tiles:,} tiles into '{pak_path}' "
        f"(x=[{x_min}..{x_max}], y=[{y_min}..{y_max}], grid={num_x}x{num_y}={grid_size:,}, "
        f"size={expected_mb:,.1f} MB)..."
    )

    start_time = time.time()
    packed = 0
    errors = 0
    src_paths = [t[2] for t in tiles]

    with open(tmp_path, "wb", buffering=4 * 1024 * 1024) as pak_f:
        # 1. Write 24-byte Header
        pak_f.write(header_bytes)
        # 2. Write Offset Table [num_x * num_y]
        pak_f.write(offset_table_bytes)
        # 3. Write alignment padding up to 512-byte sector boundary
        if padding_bytes:
            pak_f.write(padding_bytes)
        # 4. Write 3 palette sectors (512B RGB565 + 1024B BGRA8888)
        pak_f.write(rgb565_lut)
        pak_f.write(bgra_lut)

        # 5. Stream all tile payloads (65,536 bytes = 128 sectors each) in (x, y) order
        with ProcessPoolExecutor(max_workers=workers) as executor:
            chunksize = max(1, min(128, total_tiles // (workers * 4)))
            for idx, (ok, payload_or_err) in enumerate(
                executor.map(decode_tile_for_pak, src_paths, chunksize=chunksize),
                start=1,
            ):
                if ok:
                    pak_f.write(payload_or_err)
                    packed += 1
                else:
                    # Write blank tile (index 0) on error to preserve exact sector alignment
                    pak_f.write(b"\x00" * TILE_BYTES)
                    errors += 1
                    if errors <= 10:
                        print(f"\n[!] Error decoding {payload_or_err}", file=sys.stderr)

                if idx % 500 == 0 or idx == total_tiles:
                    elapsed = time.time() - start_time
                    rate = idx / max(0.001, elapsed)
                    remaining_sec = (total_tiles - idx) / max(0.001, rate)
                    pct = idx / total_tiles * 100
                    print(
                        f"  z{zoom} Progress: {idx:,}/{total_tiles:,} ({pct:5.1f}%) | "
                        f"{rate:6.1f} tiles/s | ETA: {remaining_sec / 60:4.1f}m   ",
                        end="\r",
                        flush=True,
                    )

    os.replace(tmp_path, pak_path)
    elapsed = time.time() - start_time
    print(
        f"\n[Done z{zoom}] Wrote '{pak_path}' ({expected_mb:,.1f} MB, "
        f"{packed:,} tiles in {elapsed:.1f}s, {packed / max(0.001, elapsed):.1f} tiles/s, "
        f"errors: {errors})."
    )
    return packed, errors


def collect_tasks(input_dir, output_dir, overwrite=False):
    """Scans input_dir for .png files and returns pending (src_path, dst_path) pairs."""
    pending = []
    already_done = 0

    for root, _, files in os.walk(input_dir):
        rel_dir = os.path.relpath(root, input_dir)
        target_dir = output_dir if rel_dir == "." else os.path.join(output_dir, rel_dir)

        for fname in files:
            if not fname.lower().endswith(".png"):
                continue

            src_path = os.path.join(root, fname)
            bmp_name = os.path.splitext(fname)[0] + ".bmp"
            dst_path = os.path.join(target_dir, bmp_name)

            if not overwrite and is_bmp_complete(dst_path):
                already_done += 1
            else:
                pending.append((src_path, dst_path))

    return pending, already_done


def main():
    parser = argparse.ArgumentParser(
        description="Pack PNG map tiles into contiguous 512B-sector-aligned .pak archives (tiles_z{zoom}.pak)."
    )
    parser.add_argument(
        "-i", "--input-dir",
        default=DEFAULT_INPUT_DIR,
        help=f"Source PNG tiles directory (default: {DEFAULT_INPUT_DIR})"
    )
    parser.add_argument(
        "-o", "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help=f"Destination directory for .pak files (default: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "-z", "--zoom",
        type=int,
        nargs="*",
        default=None,
        help="Optional specific zoom level(s) to pack (e.g. -z 14). Default: all zoom levels found."
    )
    parser.add_argument(
        "-j", "--workers",
        type=int,
        default=os.cpu_count() or 4,
        help="Number of parallel worker processes (default: all CPU cores)"
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing .pak (or .bmp) files instead of skipping them"
    )
    parser.add_argument(
        "--legacy-bmp",
        action="store_true",
        help="Write individual 8-bit indexed .bmp files instead of contiguous .pak files"
    )
    args = parser.parse_args()

    if not os.path.exists(args.input_dir):
        print(f"Error: Input directory '{args.input_dir}' does not exist.", file=sys.stderr)
        sys.exit(1)

    if args.legacy_bmp:
        print(f"[Scan] Scanning '{args.input_dir}' for PNG tiles...")
        t0 = time.time()
        pending, already_done = collect_tasks(args.input_dir, args.output_dir, args.overwrite)
        total_tiles = len(pending) + already_done

        print(f"[Scan] Found {total_tiles:,} tiles in {time.time() - t0:.1f}s "
              f"(Already converted: {already_done:,} | Pending: {len(pending):,}).")

        if not pending:
            print("[Done] All tiles are already converted.")
            return

        os.makedirs(args.output_dir, exist_ok=True)
        print(f"[Convert] Converting {len(pending):,} tiles to 8-bit BMP in '{args.output_dir}' "
              f"using {args.workers} workers...")

        start_time = time.time()
        converted = 0
        errors = 0

        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            chunksize = max(1, min(256, len(pending) // (args.workers * 4)))
            for idx, (ok, err_msg) in enumerate(executor.map(convert_single_tile, pending, chunksize=chunksize), start=1):
                if ok:
                    converted += 1
                else:
                    errors += 1
                    if errors <= 10:
                        print(f"\n[!] Error converting {err_msg}", file=sys.stderr)

                if idx % 500 == 0 or idx == len(pending):
                    elapsed = time.time() - start_time
                    rate = idx / max(0.001, elapsed)
                    remaining_sec = (len(pending) - idx) / max(0.001, rate)
                    current_total = already_done + idx
                    pct = current_total / total_tiles * 100
                    print(
                        f"  Progress: {current_total:,}/{total_tiles:,} ({pct:5.1f}%) | "
                        f"{rate:6.1f} tiles/s | ETA: {remaining_sec / 60:4.1f}m   ",
                        end="\r",
                        flush=True
                    )

        total_elapsed = time.time() - start_time
        print(f"\n[Done] Converted {converted:,} tiles in {total_elapsed:.1f}s "
              f"({converted / max(0.001, total_elapsed):.1f} tiles/s). Errors: {errors}.")
        return

    print(f"[Scan] Scanning '{args.input_dir}' for zoom levels and PNG tiles...")
    t0 = time.time()
    zoom_map = scan_zoom_levels(args.input_dir)
    if args.zoom:
        zoom_map = {z: tiles for z, tiles in zoom_map.items() if z in set(args.zoom)}

    if not zoom_map:
        print("No matching zoom levels / PNG tiles found.", file=sys.stderr)
        sys.exit(1)

    total_found = sum(len(t) for t in zoom_map.values())
    print(
        f"[Scan] Found {total_found:,} tiles across zoom levels {list(zoom_map.keys())} "
        f"in {time.time() - t0:.1f}s."
    )

    grand_start = time.time()
    total_packed = 0
    total_errors = 0

    for zoom, tiles in zoom_map.items():
        packed, errors = build_zoom_pak(
            zoom=zoom,
            tiles=tiles,
            output_dir=args.output_dir,
            workers=args.workers,
            overwrite=args.overwrite,
        )
        total_packed += packed
        total_errors += errors

    print(
        f"[All Done] Packed {total_packed:,} tiles in {time.time() - grand_start:.1f}s "
        f"(Errors: {total_errors})."
    )


if __name__ == "__main__":
    main()

