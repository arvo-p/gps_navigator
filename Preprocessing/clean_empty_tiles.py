import os
import time

TILES_DIR = "./tiles_dark_st7789"
EMPTY_FILE_SIZE = 921  # Standard byte size of 256x256 solid-color #121417 tile in PALETTE_16_DARK

def clean_empty_tiles():
    if not os.path.exists(TILES_DIR):
        print(f"Directory '{TILES_DIR}' does not exist.")
        return

    print(f"Scanning and deleting empty tiles in '{TILES_DIR}'...")
    t0 = time.time()
    deleted_count = 0
    scanned_count = 0

    # 1. Delete all 921-byte empty PNG tiles
    for root, dirs, files in os.walk(TILES_DIR):
        for f in files:
            if f.endswith(".png"):
                scanned_count += 1
                file_path = os.path.join(root, f)
                try:
                    if os.path.getsize(file_path) == EMPTY_FILE_SIZE:
                        os.remove(file_path)
                        deleted_count += 1
                        if deleted_count % 10000 == 0:
                            elapsed = time.time() - t0
                            rate = deleted_count / max(0.001, elapsed)
                            print(f"  Deleted {deleted_count:,} empty tiles... ({rate:.0f} files/sec)")
                except OSError as e:
                    print(f"Error removing {file_path}: {e}")

    # 2. Clean up any empty directory trees (bottom-up)
    empty_dirs_removed = 0
    for root, dirs, files in os.walk(TILES_DIR, topdown=False):
        for d in dirs:
            dir_path = os.path.join(root, d)
            try:
                if not os.listdir(dir_path):
                    os.rmdir(dir_path)
                    empty_dirs_removed += 1
            except OSError:
                pass

    total_time = time.time() - t0
    print(f"\n[Finished Cleanup in {total_time:.1f}s]")
    print(f"  Total files scanned:  {scanned_count:,}")
    print(f"  Empty tiles deleted:   {deleted_count:,}")
    print(f"  Remaining tiles:       {scanned_count - deleted_count:,}")
    print(f"  Empty dirs removed:    {empty_dirs_removed:,}")

if __name__ == "__main__":
    clean_empty_tiles()
