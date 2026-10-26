/**
 * @file map_camera.h
 * @brief Pure fixed-point / integer map camera and viewport algorithms.
 *
 * NOTE: Strictly zero float / double! Standard C99 integer math only.
 */

#ifndef MAP_CAMERA_H
#define MAP_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "geo_math.h"
#include <stdint.h>
#include <stdbool.h>

#define MAP_MAX_VISIBLE_TILES 256

/**
 * @brief Single tile positioned in screen viewport coordinates.
 */
typedef struct {
    TileCoord tile;       /**< (x, y, zoom) */
    int32_t screen_x;     /**< Pixel X position of top-left corner on screen */
    int32_t screen_y;     /**< Pixel Y position of top-left corner on screen */
    int32_t width;        /**< Rendered width (256) */
    int32_t height;       /**< Rendered height (256) */
} VisibleTile;

/**
 * @brief List of all visible tiles for the current camera frame.
 * Static fixed-capacity buffer suitable for embedded RAM.
 */
typedef struct {
    VisibleTile tiles[MAP_MAX_VISIBLE_TILES];
    int32_t count;
} VisibleTileList;

/**
 * @brief Universal map camera structure (all integer fields).
 */
typedef struct {
    GeoCoord center;              /**< Focal geographic coordinate (lat_e7, lon_e7) */
    WorldPoint32 center_wp32;     /**< 32-bit normalized world coordinate */
    int32_t zoom;                 /**< Current discrete zoom level */
    int32_t viewport_width;       /**< Viewport / screen width in pixels */
    int32_t viewport_height;      /**< Viewport / screen height in pixels */
    int32_t pan_speed_px_per_sec; /**< Base pan speed in pixels/second */
} MapCamera;

/**
 * @brief Initialize the camera with initial center, zoom, and viewport dimensions.
 */
void map_camera_init(MapCamera *cam, GeoCoord center, int32_t zoom, int32_t vp_w, int32_t vp_h);

/**
 * @brief Update camera viewport dimensions.
 */
void map_camera_set_viewport(MapCamera *cam, int32_t vp_w, int32_t vp_h);

/**
 * @brief Set a new discrete zoom level. Center coordinate is preserved.
 */
void map_camera_set_zoom(MapCamera *cam, int32_t new_zoom);

/**
 * @brief Set a new center geographic coordinate.
 */
void map_camera_set_center(MapCamera *cam, GeoCoord new_center);

/**
 * @brief Pan camera center by delta pixels in screen space.
 * @param dx_px Positive moves right (East), negative moves left (West)
 * @param dy_px Positive moves down (South), negative moves up (North)
 */
void map_camera_pan_pixels(MapCamera *cam, int32_t dx_px, int32_t dy_px);

/**
 * @brief Move camera in normalized direction over elapsed time in milliseconds.
 * @param dir_x Horizontal direction [-1, 0, +1]
 * @param dir_y Vertical direction [-1, 0, +1]
 * @param dt_ms Elapsed frame time in milliseconds
 * @param speed_multiplier Speed multiplier (e.g. 1 for normal, 2 or 3 for fast)
 */
void map_camera_move_direction(MapCamera *cam, int32_t dir_x, int32_t dir_y, int32_t dt_ms, int32_t speed_multiplier);

/**
 * @brief Convert screen pixel coordinate to geographic Lat/Lon.
 */
void map_camera_screen_to_latlon(const MapCamera *cam, int32_t sx, int32_t sy, GeoCoord *out_coord);

/**
 * @brief Convert geographic Lat/Lon to screen pixel coordinate.
 */
void map_camera_latlon_to_screen(const MapCamera *cam, GeoCoord coord, int32_t *out_sx, int32_t *out_sy);

/**
 * @brief Compute the list of tiles visible inside the current viewport.
 */
void map_camera_get_visible_tiles(const MapCamera *cam, VisibleTileList *out_list);

/**
 * @brief Get the geographic bounding box of the entire visible screen area.
 */
void map_camera_get_visible_bbox(const MapCamera *cam, GeoBBox *out_bbox);

#ifdef __cplusplus
}
#endif

#endif /* MAP_CAMERA_H */
