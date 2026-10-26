/**
 * @file map_camera.c
 * @brief Pure fixed-point / integer map camera and viewport algorithms.
 *
 * NOTE: Strictly zero float / double! Standard C99 integer math only.
 */

#include "map_camera.h"

void map_camera_init(MapCamera *cam, GeoCoord center, int32_t zoom, int32_t vp_w, int32_t vp_h) {
    if (!cam) return;

    cam->center = geo_coord_make(center.lat_e7, center.lon_e7);
    cam->zoom = (zoom < 1) ? 1 : ((zoom > 20) ? 20 : zoom);
    cam->viewport_width = (vp_w > 0) ? vp_w : 800;
    cam->viewport_height = (vp_h > 0) ? vp_h : 600;
    cam->pan_speed_px_per_sec = 400;

    geo_latlon_to_world_point32(cam->center, &cam->center_wp32);
}

void map_camera_set_viewport(MapCamera *cam, int32_t vp_w, int32_t vp_h) {
    if (!cam) return;
    if (vp_w > 0) cam->viewport_width = vp_w;
    if (vp_h > 0) cam->viewport_height = vp_h;
}

void map_camera_set_zoom(MapCamera *cam, int32_t new_zoom) {
    if (!cam) return;
    if (new_zoom < 1) new_zoom = 1;
    if (new_zoom > 20) new_zoom = 20;

    cam->zoom = new_zoom;
    /* Invariant center coordinate: re-project to 32-bit world point */
    geo_latlon_to_world_point32(cam->center, &cam->center_wp32);
}

void map_camera_set_center(MapCamera *cam, GeoCoord new_center) {
    if (!cam) return;
    cam->center = geo_coord_make(new_center.lat_e7, new_center.lon_e7);
    geo_latlon_to_world_point32(cam->center, &cam->center_wp32);
}

void map_camera_pan_pixels(MapCamera *cam, int32_t dx_px, int32_t dy_px) {
    if (!cam) return;

    int32_t shift = 24 - cam->zoom;
    int64_t delta_x32 = 0;
    int64_t delta_y32 = 0;

    if (shift >= 0) {
        delta_x32 = (int64_t)dx_px << shift;
        delta_y32 = (int64_t)dy_px << shift;
    } else {
        delta_x32 = (int64_t)dx_px >> (-shift);
        delta_y32 = (int64_t)dy_px >> (-shift);
    }

    /* Wrap horizontally around the globe */
    cam->center_wp32.x = (uint32_t)((int64_t)cam->center_wp32.x + delta_x32);

    /* Clamp vertically within valid Mercator range */
    int64_t new_y = (int64_t)cam->center_wp32.y + delta_y32;
    if (new_y < 0) new_y = 0;
    if (new_y > 4294967295LL) new_y = 4294967295LL;
    cam->center_wp32.y = (uint32_t)new_y;

    /* Update center geographic coordinate */
    geo_world_point32_to_latlon(cam->center_wp32, &cam->center);
}

void map_camera_move_direction(MapCamera *cam, int32_t dir_x, int32_t dir_y, int32_t dt_ms, int32_t speed_multiplier) {
    if (!cam || dt_ms <= 0) return;

    if (dir_x == 0 && dir_y == 0) return;

    int32_t dist = (int32_t)(((int64_t)cam->pan_speed_px_per_sec * (int64_t)speed_multiplier * (int64_t)dt_ms) / 1000LL);

    /* Normalize diagonal velocity (1 / sqrt(2) ≈ 724 / 1024) */
    if (dir_x != 0 && dir_y != 0) {
        dist = (dist * 724) >> 10;
    }

    map_camera_pan_pixels(cam, dir_x * dist, dir_y * dist);
}

void map_camera_screen_to_latlon(const MapCamera *cam, int32_t sx, int32_t sy, GeoCoord *out_coord) {
    if (!cam || !out_coord) return;

    int32_t shift = 24 - cam->zoom;
    int32_t cx = (int32_t)(cam->center_wp32.x >> shift);
    int32_t cy = (int32_t)(cam->center_wp32.y >> shift);

    int32_t tl_x = cx - cam->viewport_width / 2;
    int32_t tl_y = cy - cam->viewport_height / 2;

    int32_t wx = tl_x + sx;
    int32_t wy = tl_y + sy;

    WorldPoint32 wp;
    wp.x = (uint32_t)((int64_t)wx << shift);
    int64_t wy32 = (int64_t)wy << shift;
    if (wy32 < 0) wy32 = 0;
    if (wy32 > 4294967295LL) wy32 = 4294967295LL;
    wp.y = (uint32_t)wy32;

    geo_world_point32_to_latlon(wp, out_coord);
}

void map_camera_latlon_to_screen(const MapCamera *cam, GeoCoord coord, int32_t *out_sx, int32_t *out_sy) {
    if (!cam) return;

    WorldPoint32 wp;
    geo_latlon_to_world_point32(coord, &wp);

    int32_t shift = 24 - cam->zoom;
    int32_t cx = (int32_t)(cam->center_wp32.x >> shift);
    int32_t cy = (int32_t)(cam->center_wp32.y >> shift);

    int32_t tl_x = cx - cam->viewport_width / 2;
    int32_t tl_y = cy - cam->viewport_height / 2;

    int32_t px = (int32_t)(wp.x >> shift);
    int32_t py = (int32_t)(wp.y >> shift);

    if (out_sx) *out_sx = px - tl_x;
    if (out_sy) *out_sy = py - tl_y;
}

void map_camera_get_visible_tiles(const MapCamera *cam, VisibleTileList *out_list) {
    if (!cam || !out_list) return;

    out_list->count = 0;

    int32_t shift = 24 - cam->zoom;
    int32_t cx = (int32_t)(cam->center_wp32.x >> shift);
    int32_t cy = (int32_t)(cam->center_wp32.y >> shift);

    int32_t tl_x = cx - cam->viewport_width / 2;
    int32_t tl_y = cy - cam->viewport_height / 2;
    int32_t br_x = tl_x + cam->viewport_width;
    int32_t br_y = tl_y + cam->viewport_height;

    int32_t min_tx = tl_x >> 8;
    int32_t max_tx = br_x >> 8;
    int32_t min_ty = tl_y >> 8;
    int32_t max_ty = br_y >> 8;

    int32_t max_tile_idx = (1 << cam->zoom) - 1;

    /* Clamp vertical tile bounds */
    if (min_ty < 0) min_ty = 0;
    if (max_ty > max_tile_idx) max_ty = max_tile_idx;

    for (int32_t ty = min_ty; ty <= max_ty; ++ty) {
        for (int32_t tx = min_tx; tx <= max_tx; ++tx) {
            if (out_list->count >= MAP_MAX_VISIBLE_TILES) {
                return;
            }

            int32_t wrapped_tx = tx;
            while (wrapped_tx < 0) wrapped_tx += (1 << cam->zoom);
            while (wrapped_tx > max_tile_idx) wrapped_tx -= (1 << cam->zoom);

            VisibleTile *vt = &out_list->tiles[out_list->count++];
            vt->tile.x = wrapped_tx;
            vt->tile.y = ty;
            vt->tile.zoom = cam->zoom;

            vt->screen_x = (tx << 8) - tl_x;
            vt->screen_y = (ty << 8) - tl_y;
            vt->width = GEO_TILE_SIZE_PX;
            vt->height = GEO_TILE_SIZE_PX;
        }
    }
}

void map_camera_get_visible_bbox(const MapCamera *cam, GeoBBox *out_bbox) {
    if (!cam || !out_bbox) return;

    GeoCoord tl, br;
    map_camera_screen_to_latlon(cam, 0, 0, &tl);
    map_camera_screen_to_latlon(cam, cam->viewport_width, cam->viewport_height, &br);

    out_bbox->min_lat_e7 = br.lat_e7;
    out_bbox->max_lat_e7 = tl.lat_e7;
    out_bbox->min_lon_e7 = tl.lon_e7;
    out_bbox->max_lon_e7 = br.lon_e7;
}
