/**
 * @file geo_math.h
 * @brief Pure fixed-point / integer geographic and Mercator projection algorithms.
 *
 * NOTE: This module contains NO floating-point types (zero float, zero double),
 * NO Raylib, Windows, or OS headers. It relies exclusively on fixed-point
 * integer arithmetic (C99 <stdint.h> and <stdbool.h>) for high-speed,
 * jitter-free execution on embedded microcontrollers (Cortex-M0/M3/M4/M7).
 */

#ifndef GEO_MATH_H
#define GEO_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Coordinate scale factor: 10^7 (Micro-degrees).
 * Standard NMEA / u-blox UBX fixed-point format.
 * 1 unit ≈ 1.11 centimeters on Earth surface.
 */
#define GEO_SCALE_E7             10000000LL

#define GEO_LAT_MAX_E7           850511287L   /**< +85.0511287° Web Mercator limit */
#define GEO_LAT_MIN_E7          -850511287L   /**< -85.0511287° Web Mercator limit */
#define GEO_LON_MIN_E7         -1800000000L   /**< -180.0000000° */
#define GEO_LON_MAX_E7          1800000000L   /**< +180.0000000° */

#define GEO_TILE_SIZE_PX         256

/**
 * @brief Geographic coordinate in 10^-7 degrees (WGS84 fixed-point).
 */
typedef struct {
    int32_t lat_e7; /**< Latitude [-850511287 .. +850511287] */
    int32_t lon_e7; /**< Longitude [-1800000000 .. +1800000000] */
} GeoCoord;

/**
 * @brief Discrete tile coordinate at a given zoom level.
 */
typedef struct {
    int32_t x;    /**< Tile column index [0 .. 2^zoom - 1] */
    int32_t y;    /**< Tile row index [0 .. 2^zoom - 1] */
    int32_t zoom; /**< Zoom level */
} TileCoord;

/**
 * @brief Normalized 32-bit integer world coordinates.
 * Entire Web Mercator map is mapped to [0 .. 2^32 - 1] in both axes.
 */
typedef struct {
    uint32_t x; /**< [0 .. 2^32 - 1] representing -180° to +180° */
    uint32_t y; /**< [0 .. 2^32 - 1] representing +85.051° (top=0) to -85.051° (bottom=2^32-1) */
} WorldPoint32;

/**
 * @brief 2D pixel coordinate at current zoom level.
 */
typedef struct {
    int32_t x;
    int32_t y;
} WorldPixel;

/**
 * @brief Geographic bounding box in 10^-7 degrees.
 */
typedef struct {
    int32_t min_lat_e7;
    int32_t min_lon_e7;
    int32_t max_lat_e7;
    int32_t max_lon_e7;
} GeoBBox;

/**
 * @brief Discrete tile bounding box at a given zoom.
 */
typedef struct {
    int32_t min_x;
    int32_t max_x;
    int32_t min_y;
    int32_t max_y;
    int32_t zoom;
} TileBBox;

/* Constructors & Clamping */
GeoCoord      geo_coord_make(int32_t lat_e7, int32_t lon_e7);
TileCoord     geo_tile_coord_make(int32_t x, int32_t y, int32_t zoom);
GeoBBox       geo_bbox_make(int32_t min_lat_e7, int32_t min_lon_e7, int32_t max_lat_e7, int32_t max_lon_e7);
int32_t       geo_clamp_latitude_e7(int32_t lat_e7);
int32_t       geo_wrap_longitude_e7(int32_t lon_e7);
bool          geo_coord_is_in_bbox(GeoCoord coord, GeoBBox bbox);

/* Integer Web Mercator Projections (LUT + Fixed-Point Quadratic Interpolation) */
void          geo_latlon_to_world_point32(GeoCoord coord, WorldPoint32 *out_wp);
void          geo_world_point32_to_latlon(WorldPoint32 wp, GeoCoord *out_coord);
void          geo_latlon_to_tile(GeoCoord coord, int32_t zoom, TileCoord *out_tile, WorldPixel *out_subtile_px);
void          geo_tile_to_top_left_latlon(TileCoord tile, GeoCoord *out_coord);
void          geo_tile_to_bbox(TileCoord tile, GeoBBox *out_bbox);

/* Integer Navigational Math */
uint32_t      geo_distance_m(GeoCoord a, GeoCoord b);
int32_t       geo_bearing_deg(GeoCoord from, GeoCoord to);
uint32_t      geo_isqrt64(uint64_t n);

#ifdef __cplusplus
}
#endif

#endif /* GEO_MATH_H */
