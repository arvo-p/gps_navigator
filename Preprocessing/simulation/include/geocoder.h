/**
 * @file geocoder.h
 * @brief Forward & Reverse Geocoding Engine using Binary Map Files.
 *
 * Supports:
 *   1. Forward Geocoding (Street Name -> Coordinate):
 *      - Query format: "[street name], [town]" or "[street name]"
 *      - When a town is provided, resolves the town in towns.idx/towns.str and
 *        finds the matching street closest to that town.
 *      - When no town is provided, finds the matching street closest to a
 *        reference coordinate (e.g. current GPS position or camera center).
 *   2. Reverse Geocoding (Coordinate -> Street Name & Town):
 *      - Finds the closest named road segment (including curve geometry) to a
 *        given coordinate using local_*_nodes.bin, local_*_edges.bin, and
 *        local_*_geom.bin, then resolves its name from names.idx/names.str.
 *
 * Designed with 100% static memory and pure integer/fixed-point math for
 * seamless portability to STM32 microcontrollers.
 */

#ifndef GEOCODER_H
#define GEOCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "geo_math.h"
#include "tile_graph_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GEOCODER_NAME_MAX       128
#define GEOCODER_PATH_MAX       512
#define GEOCODER_MAX_CANDIDATES 32

/**
 * @brief Result structure for both forward and reverse geocoding lookups.
 */
typedef struct {
    bool     found;                             /**< True if a match was found */
    GeoCoord coord;                             /**< Resolved or snapped GPS coordinate (1e7) */
    uint32_t street_id;                         /**< 1-based street ID in names.idx */
    uint32_t distance_m;                        /**< Distance in meters (from query coord or town/ref) */
    char     street_name[GEOCODER_NAME_MAX];    /**< UTF-8 street name from names.str */
    char     town_name[GEOCODER_NAME_MAX];      /**< Nearest or matched UTF-8 town name from towns.str */
} GeocoderResult;

/**
 * @brief Geocoder context holding binary file paths and graph cache pointer.
 */
typedef struct {
    TileGraphCache *graph_cache;
    char names_idx_path[GEOCODER_PATH_MAX];
    char names_str_path[GEOCODER_PATH_MAX];
    char towns_idx_path[GEOCODER_PATH_MAX];
    char towns_str_path[GEOCODER_PATH_MAX];
    uint32_t street_count;
    uint32_t town_count;
} Geocoder;

/**
 * @brief Initialize the geocoder module with the binary_map_data directory and tile graph cache.
 *
 * @param geo       Pointer to Geocoder instance.
 * @param cache     Pointer to initialized TileGraphCache.
 * @param base_dir  Directory containing names.idx, names.str, towns.idx, towns.str, and local_*.bin.
 * @return true if binary index files were found and opened successfully.
 */
bool geocoder_init(Geocoder *geo, TileGraphCache *cache, const char *base_dir);

/**
 * @brief Normalize a UTF-8 street or town name to lowercase ASCII (stripping accents/punctuation).
 *        Matches the normalization used by map_binaries_gen.py when sorting names.idx.
 *
 * @param input     Raw UTF-8 string (e.g., "Rue de la Régence").
 * @param out_buf   Output buffer for normalized string (e.g., "rue de la regence").
 * @param out_size  Capacity of out_buf in bytes.
 */
void geocoder_normalize_name(const char *input, char *out_buf, size_t out_size);

/**
 * @brief Lookup a street name string from its 1-based street_id in names.idx / names.str.
 *
 * @param geo        Pointer to Geocoder instance.
 * @param street_id  1-based street index from EdgeRecord.street_id.
 * @param out_name   Output buffer for UTF-8 street name.
 * @param out_size   Capacity of out_name in bytes.
 * @return true if street_id is valid and string was read.
 */
bool geocoder_get_street_name(const Geocoder *geo, uint32_t street_id, char *out_name, size_t out_size);

/**
 * @brief Find the nearest town to a given GPS coordinate using towns.idx / towns.str.
 *
 * @param geo          Pointer to Geocoder instance.
 * @param coord        Target GPS coordinate.
 * @param out_name     Output buffer for UTF-8 town name (optional, may be NULL).
 * @param out_size     Size of out_name buffer.
 * @param out_coord    Output coordinate of the town center (optional, may be NULL).
 * @param out_dist_m   Output distance to town center in meters (optional, may be NULL).
 * @return true if a town was found.
 */
bool geocoder_find_nearest_town(const Geocoder *geo, GeoCoord coord,
                                char *out_name, size_t out_size,
                                GeoCoord *out_coord, uint32_t *out_dist_m);

/**
 * @brief Forward Geocoding: Find GPS coordinate from a street query.
 *
 * Query format:
 *   - "[street name], [town]" -> Finds the street inside/closest to [town].
 *   - "[street name]"         -> Finds the street matching [street name] closest to ref_coord.
 *
 * @param geo         Pointer to Geocoder instance.
 * @param query       Street query string (e.g. "Rue des Guillemins, Liège" or "Rue des Guillemins").
 * @param ref_coord   Reference coordinate used when no town is specified (e.g. current GPS/camera pos).
 * @param out_result  Populated with coordinate, canonical street name, town name, and distance.
 * @return true if a matching street coordinate was found.
 */
bool geocoder_street_to_coord(Geocoder *geo, const char *query, GeoCoord ref_coord,
                              GeocoderResult *out_result);

/**
 * @brief Reverse Geocoding: Find the street name (and nearest town) at a given GPS coordinate.
 *
 * @param geo         Pointer to Geocoder instance.
 * @param coord       Target GPS coordinate.
 * @param out_result  Populated with closest street name, street_id, snapped coordinate, town, and distance.
 * @return true if a named street was found near coord.
 */
bool geocoder_coord_to_street(Geocoder *geo, GeoCoord coord, GeocoderResult *out_result);

#ifdef __cplusplus
}
#endif

#endif /* GEOCODER_H */
