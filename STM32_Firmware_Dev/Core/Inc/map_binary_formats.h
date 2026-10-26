/**
 * @file map_binary_formats.h
 * @brief Binary file structure definitions matching map_binaries_gen.py export.
 */

#ifndef MAP_BINARY_FORMATS_H
#define MAP_BINARY_FORMATS_H

#include <stdint.h>

#pragma pack(push, 1)

/* Bitmask Flags for Node Features */
#define FLAG_NONE              0x0000
#define FLAG_BACKBONE_LINK     0x0001  /**< Ramp / gateway connecting local road to car highway */
#define FLAG_TRAFFIC_LIGHT     0x0002  /**< Traffic signal intersection */
#define FLAG_BARRIER_GATE      0x0004  /**< Gate / barrier / bollard */
#define FLAG_ROUNDABOUT        0x0008  /**< Roundabout node */
#define FLAG_TOLL_BOOTH        0x0010  /**< Toll booth */
#define FLAG_BORDER_CROSSING   0x0020  /**< Border control */
#define FLAG_CAR_NODE          0x0040  /**< Node is accessible by car */
#define FLAG_STROLLER_NODE     0x0080  /**< Node is accessible by stroller / pedestrian */
#define FLAG_STROLLER_GATEWAY  0x0100  /**< Gateway connecting local path/road to stroller backbone */
#define FLAG_STROLLER_BARRIER  0x0200  /**< Narrow barrier (turnstile, stile) or raised curb */

/* Bitmask Flags for Edge Access (stored in low byte of EdgeRecord.edge_flags) */
#define EDGE_CAR_ALLOWED       0x0001  /**< Edge is legal for cars in this direction */
#define EDGE_STROLLER_ALLOWED  0x0002  /**< Edge is legal for strollers in this direction */
#define EDGE_BOTH_ALLOWED      0x0003  /**< Edge is legal for both cars and strollers */
#define EDGE_CAR_BACKBONE      0x0004  /**< Edge belongs to the car backbone tier */
#define EDGE_STROLLER_BACKBONE 0x0008  /**< Edge belongs to the stroller backbone tier */

/**
 * @brief Node Record in *_nodes.bin (16 bytes, '<iiIHH')
 */
typedef struct {
    int32_t  lat_scaled;   /**< Latitude * 1e7 */
    int32_t  lon_scaled;   /**< Longitude * 1e7 */
    uint32_t first_edge;   /**< Offset index into *_edges.bin */
    uint16_t edge_count;   /**< Number of outgoing edges */
    uint16_t flags;        /**< Feature bitmask (FLAG_*) */
} NodeRecord;

/**
 * @brief Edge Record in *_edges.bin (20 bytes, '<IHHIIHH')
 */
typedef struct {
    uint32_t target_idx;   /**< Index into target *_nodes.bin */
    uint16_t cost_sec;     /**< Car travel time in seconds */
    uint16_t length_m;     /**< Road segment length in meters */
    uint32_t street_id;    /**< 1-based index in names.idx (0 if unnamed) */
    uint32_t geom_offset;  /**< Byte offset into *_geom.bin (0 if straight) */
    uint16_t geom_count;   /**< Number of intermediate curve points */
    uint16_t edge_flags;   /**< Low byte: EDGE_* access mask; High byte: stroller penalty Q3 (8=1.0x) */
} EdgeRecord;

/**
 * @brief Intermediate curve coordinate point in *_geom.bin (8 bytes, '<ii')
 */
typedef struct {
    int32_t lat_scaled;    /**< Latitude * 1e7 */
    int32_t lon_scaled;    /**< Longitude * 1e7 */
} GeomPoint;

/**
 * @brief Street name index record in names.idx (8 bytes, '<II')
 */
typedef struct {
    uint32_t str_offset;   /**< Byte offset into names.str */
    uint32_t node_id;      /**< OSM anchor node ID */
} NameIndexRecord;

/**
 * @brief Populated place / town index record in towns.idx (20 bytes, '<iiiII')
 */
typedef struct {
    int32_t  lat_scaled;   /**< Latitude * 1e7 */
    int32_t  lon_scaled;   /**< Longitude * 1e7 */
    int32_t  str_offset;   /**< Byte offset into towns.str */
    uint32_t node_id;      /**< Closest graph node ID */
    uint32_t reserved;     /**< Reserved (0) */
} TownIndexRecord;

/**
 * @brief Tile Archive (.pak) 24-byte Header ('<4s5I')
 */
#define TILE_PAK_MAGIC 0x4B415054UL /* "TPAK" in little-endian */

typedef struct {
    uint32_t magic;        /**< 0x4B415054 ("TPAK") */
    uint32_t zoom;         /**< Zoom level (12, 14, 16) */
    uint32_t x_min;        /**< Minimum tile X index */
    uint32_t x_max;        /**< Maximum tile X index */
    uint32_t y_min;        /**< Minimum tile Y index */
    uint32_t y_max;        /**< Maximum tile Y index */
} TilePakHeader;

#pragma pack(pop)

#endif /* MAP_BINARY_FORMATS_H */
