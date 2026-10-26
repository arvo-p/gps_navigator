/**
 * @file geo_math.c
 * @brief Pure fixed-point / integer geographic and Mercator projection algorithms.
 *
 * NOTE: Strictly zero float / double! Standard C99 integer math only.
 */

#include "geo_math.h"
#include "geo_lut.h"

GeoCoord geo_coord_make(int32_t lat_e7, int32_t lon_e7) {
    GeoCoord c;
    c.lat_e7 = geo_clamp_latitude_e7(lat_e7);
    c.lon_e7 = geo_wrap_longitude_e7(lon_e7);
    return c;
}

TileCoord geo_tile_coord_make(int32_t x, int32_t y, int32_t zoom) {
    TileCoord t;
    t.x = x;
    t.y = y;
    t.zoom = zoom;
    return t;
}

GeoBBox geo_bbox_make(int32_t min_lat_e7, int32_t min_lon_e7, int32_t max_lat_e7, int32_t max_lon_e7) {
    GeoBBox b;
    b.min_lat_e7 = min_lat_e7;
    b.min_lon_e7 = min_lon_e7;
    b.max_lat_e7 = max_lat_e7;
    b.max_lon_e7 = max_lon_e7;
    return b;
}

int32_t geo_clamp_latitude_e7(int32_t lat_e7) {
    if (lat_e7 > GEO_LAT_MAX_E7) return GEO_LAT_MAX_E7;
    if (lat_e7 < GEO_LAT_MIN_E7) return GEO_LAT_MIN_E7;
    return lat_e7;
}

int32_t geo_wrap_longitude_e7(int32_t lon_e7) {
    while (lon_e7 > GEO_LON_MAX_E7) lon_e7 -= 3600000000LL;
    while (lon_e7 < GEO_LON_MIN_E7) lon_e7 += 3600000000LL;
    return lon_e7;
}

bool geo_coord_is_in_bbox(GeoCoord coord, GeoBBox bbox) {
    return (coord.lat_e7 >= bbox.min_lat_e7 && coord.lat_e7 <= bbox.max_lat_e7 &&
            coord.lon_e7 >= bbox.min_lon_e7 && coord.lon_e7 <= bbox.max_lon_e7);
}

/**
 * @brief Fast 64-bit integer square root (binary restoring method).
 * 0 byte table overhead, ~16-32 instructions on ARM Cortex-M.
 */
uint32_t geo_isqrt64(uint64_t n) {
    if (n == 0) return 0;
    uint64_t root = 0;
    uint64_t bit = 1ULL << 62;

    while (bit > n) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)root;
}

void geo_latlon_to_world_point32(GeoCoord coord, WorldPoint32 *out_wp) {
    if (!out_wp) return;

    int32_t lon = geo_wrap_longitude_e7(coord.lon_e7);
    int32_t lat = geo_clamp_latitude_e7(coord.lat_e7);

    /* ------------------------------------------------------------- */
    /* 1. Longitude -> X32:                                          */
    /* Shift [-180..+180] deg to [0..360] deg.                       */
    /* (lon + 180e7) * 2^32 / 360e7 = (lon + 180e7) * 4194304 / 3515625 */
    /* ------------------------------------------------------------- */
    uint64_t lon_shifted = (uint64_t)((int64_t)lon + 1800000000LL);
    out_wp->x = (uint32_t)((lon_shifted * 4194304ULL) / 3515625ULL);

    /* ------------------------------------------------------------- */
    /* 2. Latitude -> Y32:                                           */
    /* Uses symmetric Forward LUT with fixed-point quadratic inter-  */
    /* polation.                                                     */
    /* ------------------------------------------------------------- */
    int is_south = 0;
    if (lat < 0) {
        is_south = 1;
        lat = -lat;
    }

    int32_t idx = lat / GEO_LUT_STEP_LAT_E7;
    if (idx >= GEO_LUT_N) idx = GEO_LUT_N - 1;

    int64_t rem = (int64_t)(lat - idx * GEO_LUT_STEP_LAT_E7);
    int64_t t_q16 = (rem * 65536LL) / GEO_LUT_STEP_LAT_E7;

    int64_t y0 = (int64_t)GEO_MERCATOR_FORWARD_LUT[idx];
    int64_t y1 = (int64_t)GEO_MERCATOR_FORWARD_LUT[idx + 1];
    int64_t y2 = (int64_t)GEO_MERCATOR_FORWARD_LUT[idx + 2];

    int64_t delta1 = y1 - y0;
    int64_t delta2 = y2 - 2 * y1 + y0;

    int64_t term1 = (t_q16 * delta1) >> 16;
    int64_t t_minus_1 = t_q16 - 65536LL;
    int64_t term2 = (((t_q16 * t_minus_1) / 2LL) * delta2) >> 32;

    int64_t y_north = y0 + term1 + term2;

    if (is_south) {
        /* Reflect across equator (2^31) */
        out_wp->y = (uint32_t)(4294967296ULL - (uint64_t)y_north);
    } else {
        out_wp->y = (uint32_t)y_north;
    }
}

void geo_world_point32_to_latlon(WorldPoint32 wp, GeoCoord *out_coord) {
    if (!out_coord) return;

    /* ------------------------------------------------------------- */
    /* 1. X32 -> Longitude:                                          */
    /* lon_e7 = (X32 * 3515625 / 4194304) - 180e7                    */
    /* ------------------------------------------------------------- */
    uint64_t num = (uint64_t)wp.x * 3515625ULL + 2097152ULL;
    int64_t lon_shifted = (int64_t)(num / 4194304ULL);
    out_coord->lon_e7 = geo_wrap_longitude_e7((int32_t)(lon_shifted - 1800000000LL));

    /* ------------------------------------------------------------- */
    /* 2. Y32 -> Latitude:                                           */
    /* Uses symmetric Inverse LUT with quadratic interpolation.       */
    /* ------------------------------------------------------------- */
    const uint32_t EQUATOR_Y32 = 2147483648U;
    int is_south = 0;
    uint32_t dist_from_top = 0;

    if (wp.y > EQUATOR_Y32) {
        is_south = 1;
        dist_from_top = (uint32_t)(4294967296ULL - (uint64_t)wp.y);
    } else {
        dist_from_top = wp.y;
    }

    int32_t idx = (int32_t)(dist_from_top / GEO_LUT_INV_STEP_Y32);
    if (idx >= GEO_LUT_N) idx = GEO_LUT_N - 1;

    int64_t rem = (int64_t)(dist_from_top - (uint32_t)idx * GEO_LUT_INV_STEP_Y32);
    int64_t t_q16 = (rem * 65536LL) / GEO_LUT_INV_STEP_Y32;

    int64_t l0 = (int64_t)GEO_MERCATOR_INVERSE_LUT[idx];
    int64_t l1 = (int64_t)GEO_MERCATOR_INVERSE_LUT[idx + 1];
    int64_t l2 = (int64_t)GEO_MERCATOR_INVERSE_LUT[idx + 2];

    int64_t delta1 = l1 - l0;
    int64_t delta2 = l2 - 2 * l1 + l0;

    int64_t term1 = (t_q16 * delta1) >> 16;
    int64_t t_minus_1 = t_q16 - 65536LL;
    int64_t term2 = (((t_q16 * t_minus_1) / 2LL) * delta2) >> 32;

    int32_t lat_val = (int32_t)(l0 + term1 + term2);

    if (is_south) {
        out_coord->lat_e7 = geo_clamp_latitude_e7(-lat_val);
    } else {
        out_coord->lat_e7 = geo_clamp_latitude_e7(lat_val);
    }
}

void geo_latlon_to_tile(GeoCoord coord, int32_t zoom, TileCoord *out_tile, WorldPixel *out_subtile_px) {
    WorldPoint32 wp;
    geo_latlon_to_world_point32(coord, &wp);

    if (zoom < 0) zoom = 0;
    if (zoom > 24) zoom = 24;

    /* Tile coordinates are the top bits of 32-bit world space */
    int32_t shift_tile = 32 - zoom;
    int32_t tx = (int32_t)(wp.x >> shift_tile);
    int32_t ty = (int32_t)(wp.y >> shift_tile);

    if (out_tile) {
        out_tile->x = tx;
        out_tile->y = ty;
        out_tile->zoom = zoom;
    }

    if (out_subtile_px) {
        /* Intra-tile pixel is bits [shift_tile - 1 .. shift_tile - 8] */
        int32_t shift_px = 24 - zoom;
        if (shift_px >= 0) {
            out_subtile_px->x = (int32_t)((wp.x >> shift_px) & 0xFF);
            out_subtile_px->y = (int32_t)((wp.y >> shift_px) & 0xFF);
        } else {
            out_subtile_px->x = 0;
            out_subtile_px->y = 0;
        }
    }
}

void geo_tile_to_top_left_latlon(TileCoord tile, GeoCoord *out_coord) {
    if (!out_coord) return;

    WorldPoint32 wp;
    wp.x = (uint32_t)tile.x << (32 - tile.zoom);
    wp.y = (uint32_t)tile.y << (32 - tile.zoom);

    geo_world_point32_to_latlon(wp, out_coord);
}

void geo_tile_to_bbox(TileCoord tile, GeoBBox *out_bbox) {
    if (!out_bbox) return;

    WorldPoint32 wp_tl, wp_br;
    wp_tl.x = (uint32_t)tile.x << (32 - tile.zoom);
    wp_tl.y = (uint32_t)tile.y << (32 - tile.zoom);

    wp_br.x = (uint32_t)(tile.x + 1) << (32 - tile.zoom);
    wp_br.y = (uint32_t)(tile.y + 1) << (32 - tile.zoom);

    GeoCoord top_left, bottom_right;
    geo_world_point32_to_latlon(wp_tl, &top_left);
    geo_world_point32_to_latlon(wp_br, &bottom_right);

    out_bbox->min_lat_e7 = bottom_right.lat_e7;
    out_bbox->max_lat_e7 = top_left.lat_e7;
    out_bbox->min_lon_e7 = top_left.lon_e7;
    out_bbox->max_lon_e7 = bottom_right.lon_e7;
}

uint32_t geo_distance_m(GeoCoord a, GeoCoord b) {
    int64_t dlat = (int64_t)(b.lat_e7 - a.lat_e7);
    int64_t dlon = (int64_t)(b.lon_e7 - a.lon_e7);

    int32_t mean_lat = (a.lat_e7 + b.lat_e7) / 2;
    if (mean_lat < 0) mean_lat = -mean_lat;
    int32_t deg = mean_lat / 10000000;
    if (deg > 90) deg = 90;

    int32_t cos_q15 = GEO_COS_LUT_Q15[deg];
    int64_t dlon_scaled = (dlon * cos_q15) >> 15;

    uint64_t sum_sq = (uint64_t)(dlat * dlat + dlon_scaled * dlon_scaled);
    uint32_t dist_e7 = geo_isqrt64(sum_sq);

    /* 1e-7 deg = 111139 / 10000000 meters */
    return (uint32_t)(((uint64_t)dist_e7 * 111139ULL) / 10000000ULL);
}

int32_t geo_bearing_deg(GeoCoord from, GeoCoord to) {
    int64_t dlat = (int64_t)(to.lat_e7 - from.lat_e7);
    int64_t dlon = (int64_t)(to.lon_e7 - from.lon_e7);

    if (dlat == 0 && dlon == 0) return 0;

    int32_t mean_lat = (from.lat_e7 + to.lat_e7) / 2;
    if (mean_lat < 0) mean_lat = -mean_lat;
    int32_t deg = mean_lat / 10000000;
    if (deg > 90) deg = 90;

    int64_t dy = (dlon * GEO_COS_LUT_Q15[deg]) >> 15;
    int64_t dx = dlat;

    int64_t ax = (dx < 0) ? -dx : dx;
    int64_t ay = (dy < 0) ? -dy : dy;

    int32_t angle_tenths = 0;
    if (ax >= ay) {
        if (ax == 0) return 0;
        int32_t idx = (int32_t)((ay * 64LL) / ax);
        if (idx > 64) idx = 64;
        angle_tenths = GEO_ATAN_TENTHS_LUT[idx];
    } else {
        int32_t idx = (int32_t)((ax * 64LL) / ay);
        if (idx > 64) idx = 64;
        angle_tenths = 900 - GEO_ATAN_TENTHS_LUT[idx];
    }

    if (dx < 0 && dy >= 0) angle_tenths = 1800 - angle_tenths;
    else if (dx < 0 && dy < 0) angle_tenths = 1800 + angle_tenths;
    else if (dx >= 0 && dy < 0) angle_tenths = 3600 - angle_tenths;

    int32_t result_deg = (angle_tenths + 5) / 10;
    return result_deg % 360;
}
