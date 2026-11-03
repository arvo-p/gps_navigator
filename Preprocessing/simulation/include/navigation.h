/**
 * @file navigation.h
 * @brief Turn-by-Turn Navigation & On-the-Fly "Lego" Direction String Builder.
 *
 * Designed for embedded systems (STM32 / Raylib simulation):
 *   - Zero dynamic allocation (no malloc/free)
 *   - Pure integer/fixed-point arithmetic (no float/double)
 *   - Generates the upcoming maneuver on the fly as the user moves along the route
 *   - Caches only the single active target street name so SD card string lookups
 *     occur at most once per intersection
 *   - Builds human-readable direction strings on demand ("Lego" assembly)
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "geo_math.h"
#include "pathfinder.h"
#include "geocoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_STREET_CACHE_MAX 64

/**
 * @brief Turn-by-turn maneuver classification.
 */
typedef enum {
    NAV_NONE = 0,
    NAV_CONTINUE_STRAIGHT,  /**< Heading change < 15° across an intersection / new street */
    NAV_KEEP_LEFT,          /**< Fork / slight veer left (15° .. 44°) */
    NAV_KEEP_RIGHT,         /**< Fork / slight veer right (15° .. 44°) */
    NAV_TURN_LEFT,          /**< Standard left turn (45° .. 135°) */
    NAV_TURN_RIGHT,         /**< Standard right turn (45° .. 135°) */
    NAV_SHARP_LEFT,         /**< Hard left turn / switchback (> 135°) */
    NAV_SHARP_RIGHT,        /**< Hard right turn / switchback (> 135°) */
    NAV_ROUNDABOUT,         /**< Roundabout entry + exit count */
    NAV_ARRIVE              /**< Destination reached / ahead */
} NavAction;

/**
 * @brief Compact 16-byte upcoming maneuver descriptor ("Lego" data blocks).
 */
typedef struct {
    NavAction action;           /**< Maneuver action type */
    uint8_t   exit_num;         /**< 1-based exit number when action == NAV_ROUNDABOUT */
    int16_t   angle_deg;        /**< Signed turn angle in degrees (+left, -right) */
    uint32_t  dist_m;           /**< Remaining distance in meters to this maneuver */
    uint32_t  next_street_id;   /**< 1-based street_id of the road entered after maneuver */
    int32_t   maneuver_seg_idx; /**< Segment index immediately following the maneuver */
} NavNextStep;

/**
 * @brief Lightweight live navigation tracker with 1-entry street name cache.
 */
typedef struct {
    int32_t     current_seg_idx;                       /**< Active segment index along route */
    uint32_t    cached_street_id;                      /**< Street ID currently cached in RAM */
    char        cached_street_name[NAV_STREET_CACHE_MAX]; /**< Cached UTF-8 street name */
    NavNextStep next_step;                             /**< Upcoming maneuver descriptor */
} NavState;

/**
 * @brief Initialize or reset live navigation state for a new route.
 */
void nav_init(NavState *nav);

/**
 * @brief Classify the maneuver at the end of segment `seg_idx` (transitioning to `seg_idx + 1`).
 *
 * @param route             Active route result.
 * @param seg_idx           Index of the incoming segment [0 .. route->segment_count - 1].
 * @param out_angle_deg     Signed turn angle in degrees (+left, -right).
 * @param out_exit_num      Roundabout exit count (1..N) if NAV_ROUNDABOUT, else 0.
 * @param out_next_seg_idx  Segment index immediately after the maneuver (skips internal roundabout edges).
 * @return Classified NavAction (or NAV_NONE if the road simply continues on the same street).
 */
NavAction nav_classify_transition(const RouteResult *route, int32_t seg_idx,
                                  int16_t *out_angle_deg, uint8_t *out_exit_num,
                                  int32_t *out_next_seg_idx);

/**
 * @brief Compute the next upcoming maneuver on the fly from the user's current GPS coordinate.
 *
 * @param nav      Navigation tracker state (updates current_seg_idx, next_step, and 1-item street cache).
 * @param route    Active RouteResult.
 * @param geo      Optional Geocoder instance to resolve next_street_id once when it changes (may be NULL).
 * @param gps_pos  Current GPS or simulated position.
 * @return true if route is valid and nav->next_step was updated.
 */
bool nav_update(NavState *nav, const RouteResult *route, const Geocoder *geo, GeoCoord gps_pos);

/**
 * @brief Return the short action label for a maneuver (e.g. "Turn Left", "Keep Right").
 */
const char* nav_action_label(NavAction action);

/**
 * @brief "Lego" formatter: builds the human-readable direction string right before displaying/printing.
 *
 * Examples:
 *   - "In 180 m, Turn Left onto Rue des Guillemins"
 *   - "In 60 m, At the roundabout, take exit 2 onto Boulevard d'Avroy"
 *   - "Keep Right onto Avenue Blonden" (when dist_m < 15 m)
 *   - "In 45 m, Arrive at destination"
 *
 * @param step         Compact maneuver step descriptor.
 * @param street_name  Target street name string (e.g. nav->cached_street_name, or NULL/"").
 * @param out_buf      Destination character buffer.
 * @param buf_size     Capacity of out_buf in bytes.
 */
void nav_format_instruction(const NavNextStep *step, const char *street_name,
                            char *out_buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H */
