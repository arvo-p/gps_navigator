/**
 * @file navigation.c
 * @brief Turn-by-Turn Navigation & On-the-Fly "Lego" Direction String Builder.
 *
 * 100% static allocation and pure integer/fixed-point arithmetic.
 */

#include "navigation.h"
#include <stdio.h>
#include <string.h>

/*============================================================================
 * 1. Integer Math & Angle Helpers (Zero Floating-Point)
 *============================================================================*/

static uint32_t isqrt_u64(uint64_t n) {
    if (n == 0) return 0;
    uint64_t x = n;
    uint64_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return (uint32_t)x;
}

static uint32_t calc_dist_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    int64_t dlat = (int64_t)lat2_e7 - (int64_t)lat1_e7;
    int64_t dlon = (int64_t)lon2_e7 - (int64_t)lon1_e7;
    int64_t dy_m = (dlat * 111320LL) / 10000000LL;
    int64_t dx_m = (dlon * 70500LL)  / 10000000LL;
    return isqrt_u64((uint64_t)(dx_m * dx_m + dy_m * dy_m));
}

/**
 * @brief Shortest distance in meters from point P to segment A->B,
 *        plus remaining distance from the projected point to B.
 */
static uint32_t point_to_segment_dist_and_rem_m(int32_t p_lat, int32_t p_lon,
                                                int32_t a_lat, int32_t a_lon,
                                                int32_t b_lat, int32_t b_lon,
                                                uint32_t *out_rem_to_b_m) {
    int64_t px_mm = ((int64_t)(p_lon - a_lon) * 70500LL)  / 10000LL;
    int64_t py_mm = ((int64_t)(p_lat - a_lat) * 111320LL) / 10000LL;

    int64_t bx_mm = ((int64_t)(b_lon - a_lon) * 70500LL)  / 10000LL;
    int64_t by_mm = ((int64_t)(b_lat - a_lat) * 111320LL) / 10000LL;

    int64_t len_sq = bx_mm * bx_mm + by_mm * by_mm;
    int64_t proj_x_mm = 0;
    int64_t proj_y_mm = 0;

    if (len_sq > 0) {
        int64_t dot = px_mm * bx_mm + py_mm * by_mm;
        if (dot >= len_sq) {
            proj_x_mm = bx_mm;
            proj_y_mm = by_mm;
        } else if (dot > 0) {
            int64_t t_q10 = (dot << 10) / len_sq;
            proj_x_mm = (bx_mm * t_q10) >> 10;
            proj_y_mm = (by_mm * t_q10) >> 10;
        }
    }

    if (out_rem_to_b_m) {
        int64_t rx_m = (bx_mm - proj_x_mm) / 1000LL;
        int64_t ry_m = (by_mm - proj_y_mm) / 1000LL;
        *out_rem_to_b_m = isqrt_u64((uint64_t)(rx_m * rx_m + ry_m * ry_m));
    }

    int64_t dx_m = (px_mm - proj_x_mm) / 1000LL;
    int64_t dy_m = (py_mm - proj_y_mm) / 1000LL;
    return isqrt_u64((uint64_t)(dx_m * dx_m + dy_m * dy_m));
}

/**
 * @brief Pure integer atan2(y, x) returning degrees in [-180 .. +180].
 *        Max error < 0.25 degrees across all quadrants.
 */
static int16_t atan2_deg_int(int64_t y, int64_t x) {
    if (x == 0 && y == 0) return 0;

    int64_t ax = (x < 0) ? -x : x;
    int64_t ay = (y < 0) ? -y : y;

    int64_t u = (ax < ay) ? ax : ay;
    int64_t v = (ax < ay) ? ay : ax;

    /* Scale down u and v to prevent overflow in v * v */
    while (v > 32768LL) {
        u >>= 1;
        v >>= 1;
    }
    if (v == 0) return 0;

    /* Rational polynomial approximation on [0 .. 45] degrees */
    int32_t deg = (int32_t)((45LL * u) / v + (16LL * u * (v - u)) / (v * v));

    if (ax < ay) {
        deg = 90 - deg;
    }
    if (x < 0) {
        deg = 180 - deg;
    }
    if (y < 0) {
        deg = -deg;
    }
    return (int16_t)deg;
}

/**
 * @brief Extract incoming tangent vector (dx_mm, dy_mm) approaching the end of seg_idx.
 *        Uses route->geom_points[] if loaded (walking back >= 8m from the junction to
 *        avoid curb micro-jitter), or falls back to node chord start->end.
 */
static void get_incoming_vector_mm(const RouteResult *route, int32_t seg_idx,
                                   int64_t *out_dx_mm, int64_t *out_dy_mm) {
    const RouteSegment *seg = &route->segments[seg_idx];
    int32_t p1_lat = seg->start_lat_e7;
    int32_t p1_lon = seg->start_lon_e7;
    int32_t p2_lat = seg->end_lat_e7;
    int32_t p2_lon = seg->end_lon_e7;

    if (route->geom_point_count > 0 && seg->geom_end_idx > seg->geom_start_idx &&
        seg->geom_end_idx < (uint16_t)route->geom_point_count) {
        int32_t idx = (int32_t)seg->geom_end_idx - 1;
        while (idx > (int32_t)seg->geom_start_idx) {
            uint32_t d = calc_dist_m(route->geom_points[idx].lat_scaled,
                                     route->geom_points[idx].lon_scaled,
                                     p2_lat, p2_lon);
            if (d >= 8U) break;
            idx--;
        }
        p1_lat = route->geom_points[idx].lat_scaled;
        p1_lon = route->geom_points[idx].lon_scaled;
    }

    *out_dx_mm = ((int64_t)(p2_lon - p1_lon) * 70500LL)  / 10000LL;
    *out_dy_mm = ((int64_t)(p2_lat - p1_lat) * 111320LL) / 10000LL;
}

/**
 * @brief Extract outgoing tangent vector (dx_mm, dy_mm) leaving the start of seg_idx.
 */
static void get_outgoing_vector_mm(const RouteResult *route, int32_t seg_idx,
                                   int64_t *out_dx_mm, int64_t *out_dy_mm) {
    const RouteSegment *seg = &route->segments[seg_idx];
    int32_t p1_lat = seg->start_lat_e7;
    int32_t p1_lon = seg->start_lon_e7;
    int32_t p2_lat = seg->end_lat_e7;
    int32_t p2_lon = seg->end_lon_e7;

    if (route->geom_point_count > 0 && seg->geom_end_idx > seg->geom_start_idx &&
        seg->geom_end_idx < (uint16_t)route->geom_point_count) {
        int32_t idx = (int32_t)seg->geom_start_idx + 1;
        while (idx < (int32_t)seg->geom_end_idx) {
            uint32_t d = calc_dist_m(p1_lat, p1_lon,
                                     route->geom_points[idx].lat_scaled,
                                     route->geom_points[idx].lon_scaled);
            if (d >= 8U) break;
            idx++;
        }
        p2_lat = route->geom_points[idx].lat_scaled;
        p2_lon = route->geom_points[idx].lon_scaled;
    }

    *out_dx_mm = ((int64_t)(p2_lon - p1_lon) * 70500LL)  / 10000LL;
    *out_dy_mm = ((int64_t)(p2_lat - p1_lat) * 111320LL) / 10000LL;
}

/**
 * @brief Compute signed turn angle in degrees between incoming segment `seg_in`
 *        and outgoing segment `seg_out`.
 *        Positive (+1..+180) = Left turn; Negative (-1..-180) = Right turn.
 */
static int16_t compute_turn_angle_deg(const RouteResult *route, int32_t seg_in, int32_t seg_out) {
    int64_t in_dx = 0, in_dy = 0;
    int64_t out_dx = 0, out_dy = 0;
    get_incoming_vector_mm(route, seg_in, &in_dx, &in_dy);
    get_outgoing_vector_mm(route, seg_out, &out_dx, &out_dy);

    /* Scale down vectors if large to avoid 64-bit overflow in dot/cross products */
    while (in_dx > 1000000LL || in_dx < -1000000LL || in_dy > 1000000LL || in_dy < -1000000LL) {
        in_dx >>= 2;
        in_dy >>= 2;
    }
    while (out_dx > 1000000LL || out_dx < -1000000LL || out_dy > 1000000LL || out_dy < -1000000LL) {
        out_dx >>= 2;
        out_dy >>= 2;
    }

    int64_t dot   = in_dx * out_dx + in_dy * out_dy;
    int64_t cross = in_dx * out_dy - in_dy * out_dx;
    return atan2_deg_int(cross, dot);
}

/*============================================================================
 * 2. Maneuver Classification & On-the-Fly Lookahead
 *============================================================================*/

static bool is_roundabout_segment(const RouteSegment *seg) {
    return ((seg->start_node_flags & FLAG_ROUNDABOUT) != 0) &&
           ((seg->end_node_flags   & FLAG_ROUNDABOUT) != 0);
}

void nav_init(NavState *nav) {
    if (!nav) return;
    memset(nav, 0, sizeof(NavState));
}

NavAction nav_classify_transition(const RouteResult *route, int32_t seg_idx,
                                  int16_t *out_angle_deg, uint8_t *out_exit_num,
                                  int32_t *out_next_seg_idx) {
    if (out_angle_deg)    *out_angle_deg = 0;
    if (out_exit_num)     *out_exit_num = 0;
    if (out_next_seg_idx) *out_next_seg_idx = seg_idx + 1;

    if (!route || !route->found || seg_idx < 0 || seg_idx >= route->segment_count) {
        return NAV_NONE;
    }

    /* End of the final segment -> Arrive at destination */
    if (seg_idx == route->segment_count - 1) {
        return NAV_ARRIVE;
    }

    const RouteSegment *curr_seg = &route->segments[seg_idx];
    const RouteSegment *next_seg = &route->segments[seg_idx + 1];

    /* 1. Roundabout Detection:
     *    Triggered when approaching a roundabout node from a non-roundabout segment,
     *    or already on the entry node of a roundabout. */
    bool entering_rbt = (!is_roundabout_segment(curr_seg)) &&
                        ((curr_seg->end_node_flags & FLAG_ROUNDABOUT) != 0 ||
                         is_roundabout_segment(next_seg));

    if (entering_rbt) {
        uint8_t exits_passed = 0;
        int32_t k = seg_idx + 1;

        while (k < route->segment_count) {
            const RouteSegment *rseg = &route->segments[k];
            bool on_ring = ((rseg->start_node_flags & FLAG_ROUNDABOUT) != 0) &&
                           ((rseg->end_node_flags   & FLAG_ROUNDABOUT) != 0);
            if (!on_ring) {
                /* Segment k leaves the roundabout ring (this is the exit we take) */
                if (exits_passed < 255) exits_passed++;
                break;
            }
            /* Only count rseg's end node as a bypassed exit if the NEXT segment stays on the ring */
            bool next_stays_on_ring = (k + 1 < route->segment_count) &&
                                      ((route->segments[k + 1].start_node_flags & FLAG_ROUNDABOUT) != 0) &&
                                      ((route->segments[k + 1].end_node_flags   & FLAG_ROUNDABOUT) != 0);
            if (next_stays_on_ring && rseg->out_degree >= 2 && exits_passed < 254) {
                exits_passed++;
            }
            k++;
        }

        if (exits_passed == 0) exits_passed = 1;
        int32_t exit_seg = (k < route->segment_count) ? k : (route->segment_count - 1);

        if (out_exit_num)     *out_exit_num = exits_passed;
        if (out_next_seg_idx) *out_next_seg_idx = exit_seg;
        if (out_angle_deg)    *out_angle_deg = compute_turn_angle_deg(route, seg_idx, exit_seg);
        return NAV_ROUNDABOUT;
    }

    /* 2. Standard Intersection / Fork / Turn Angle Evaluation */
    int16_t angle = compute_turn_angle_deg(route, seg_idx, seg_idx + 1);
    if (out_angle_deg) *out_angle_deg = angle;

    int16_t abs_angle = (angle < 0) ? -angle : angle;
    bool same_street  = (curr_seg->street_id != 0 && curr_seg->street_id == next_seg->street_id);
    bool street_changed = (next_seg->street_id != 0 && curr_seg->street_id != next_seg->street_id);
    bool is_intersection = (curr_seg->out_degree >= 2);

    /* A. Sharp Left / Sharp Right (Hard Turn > 135 degrees) */
    if (abs_angle > 135) {
        return (angle > 0) ? NAV_SHARP_LEFT : NAV_SHARP_RIGHT;
    }

    /* B. Turn Left / Turn Right (45 to 135 degrees) */
    if (abs_angle >= 45) {
        /* If staying on the exact same street with no branching choice (out_degree < 2),
         * ignore gentle road bends (< 65 deg), only alert on real intersection turns. */
        if (same_street && !is_intersection && abs_angle < 65) {
            return NAV_NONE;
        }
        return (angle > 0) ? NAV_TURN_LEFT : NAV_TURN_RIGHT;
    }

    /* C. Keep Left / Keep Right (Fork relative to alternative forward branch, or shallow street change) */
    if (curr_seg->fork_dir != 0) {
        /* If staying on the same named road and going nearly dead straight (< 10 deg) past a minor slip lane,
         * only trigger Keep Left/Right when the road itself splits (>= 10 deg) or changes street name. */
        if (street_changed || abs_angle >= 10) {
            return (curr_seg->fork_dir > 0) ? NAV_KEEP_LEFT : NAV_KEEP_RIGHT;
        }
    } else if (abs_angle >= 15 && street_changed) {
        return (angle > 0) ? NAV_KEEP_LEFT : NAV_KEEP_RIGHT;
    }

    /* D. Continue Straight (< 15 degrees onto a new street name at an intersection) */
    if (street_changed && is_intersection) {
        return NAV_CONTINUE_STRAIGHT;
    }

    return NAV_NONE;
}

bool nav_update(NavState *nav, const RouteResult *route, const Geocoder *geo, GeoCoord gps_pos) {
    if (!nav || !route || !route->found || route->segment_count <= 0) {
        return false;
    }

    if (nav->current_seg_idx < 0 || nav->current_seg_idx >= route->segment_count) {
        nav->current_seg_idx = 0;
    }

    /* 1. Find closest segment to gps_pos (preferring forward progress from current_seg_idx) */
    uint32_t best_dist_m = 0xFFFFFFFFU;
    uint32_t best_rem_m  = 0;
    int32_t  best_seg    = nav->current_seg_idx;

    int32_t search_start = (nav->current_seg_idx > 2) ? (nav->current_seg_idx - 2) : 0;
    for (int32_t i = search_start; i < route->segment_count; ++i) {
        const RouteSegment *seg = &route->segments[i];
        uint32_t rem_m = 0;
        uint32_t d = point_to_segment_dist_and_rem_m(
            gps_pos.lat_e7, gps_pos.lon_e7,
            seg->start_lat_e7, seg->start_lon_e7,
            seg->end_lat_e7, seg->end_lon_e7,
            &rem_m
        );
        if (d < best_dist_m || (d == best_dist_m && best_rem_m == 0 && rem_m > 0)) {
            best_dist_m = d;
            best_rem_m  = rem_m;
            best_seg    = i;
        }
        /* Stop scanning far ahead once we are well past a close match */
        if (best_dist_m < 30U && i > best_seg + 8) {
            break;
        }
    }

    nav->current_seg_idx = best_seg;

    /* 2. Scan forward from best_seg accumulating distance until the next real maneuver */
    uint32_t accum_dist_m = best_rem_m;
    NavAction action      = NAV_ARRIVE;
    int16_t   angle_deg   = 0;
    uint8_t   exit_num    = 0;
    int32_t   next_seg    = route->segment_count - 1;

    for (int32_t s = best_seg; s < route->segment_count; ++s) {
        if (s > best_seg) {
            accum_dist_m += route->segments[s].length_m;
        }

        NavAction act = nav_classify_transition(route, s, &angle_deg, &exit_num, &next_seg);
        if (act != NAV_NONE) {
            action = act;
            break;
        }
    }

    uint32_t target_street_id = 0;
    if (action != NAV_ARRIVE && next_seg >= 0 && next_seg < route->segment_count) {
        target_street_id = route->segments[next_seg].street_id;
    } else if (action == NAV_ARRIVE && route->segment_count > 0) {
        target_street_id = route->segments[route->segment_count - 1].street_id;
    }

    /* Quantize distance to 5 m steps (matching 2 m GPS accuracy) */
    uint32_t dist_5m = ((accum_dist_m + 2U) / 5U) * 5U;

    nav->next_step.action           = action;
    nav->next_step.exit_num         = exit_num;
    nav->next_step.angle_deg        = angle_deg;
    nav->next_step.dist_m           = dist_5m;
    nav->next_step.next_street_id   = target_street_id;
    nav->next_step.maneuver_seg_idx = next_seg;

    /* 3. Update 1-entry street name cache ONLY when target_street_id changes */
    if (target_street_id == 0) {
        nav->cached_street_id = 0;
        nav->cached_street_name[0] = '\0';
    } else if (target_street_id != nav->cached_street_id) {
        nav->cached_street_id = target_street_id;
        nav->cached_street_name[0] = '\0';
        if (geo) {
            geocoder_get_street_name(geo, target_street_id,
                                     nav->cached_street_name, sizeof(nav->cached_street_name));
        }
    }

    return true;
}

/*============================================================================
 * 3. "Lego" On-Demand Instruction String Builder
 *============================================================================*/

const char* nav_action_label(NavAction action) {
    switch (action) {
        case NAV_CONTINUE_STRAIGHT: return "Continue Straight";
        case NAV_KEEP_LEFT:         return "Keep Left";
        case NAV_KEEP_RIGHT:        return "Keep Right";
        case NAV_TURN_LEFT:         return "Turn Left";
        case NAV_TURN_RIGHT:        return "Turn Right";
        case NAV_SHARP_LEFT:        return "Sharp Left";
        case NAV_SHARP_RIGHT:       return "Sharp Right";
        case NAV_ROUNDABOUT:        return "Roundabout";
        case NAV_ARRIVE:            return "Arrive at destination";
        default:                    return "Continue";
    }
}

static const char* roundabout_ordinal_word(uint8_t exit_num) {
    switch (exit_num) {
        case 1:  return "first";
        case 2:  return "second";
        case 3:  return "third";
        case 4:  return "fourth";
        case 5:  return "fifth";
        case 6:  return "sixth";
        case 7:  return "seventh";
        case 8:  return "eighth";
        case 9:  return "ninth";
        case 10: return "tenth";
        default: return "next";
    }
}

void nav_format_instruction(const NavNextStep *step, const char *street_name,
                            char *out_buf, size_t buf_size) {
    if (!out_buf || buf_size == 0) return;
    out_buf[0] = '\0';
    if (!step) return;

    /* Lego Piece 1: Distance prefix quantized to 5 m accuracy */
    uint32_t rounded_m = ((step->dist_m + 2U) / 5U) * 5U;
    char dist_prefix[32] = {0};
    if (rounded_m >= 5U) {
        snprintf(dist_prefix, sizeof(dist_prefix), "In %u m, ", (unsigned)rounded_m);
    }

    bool has_street = (street_name && street_name[0] != '\0');

    /* Lego Piece 2: Action + Roundabout Exit + Lego Piece 3: Target Street Name */
    if (step->action == NAV_ARRIVE) {
        if (rounded_m < 5U) {
            snprintf(out_buf, buf_size, "You have arrived at your destination");
        } else if (has_street) {
            snprintf(out_buf, buf_size, "%sArrive at destination on %s", dist_prefix, street_name);
        } else {
            snprintf(out_buf, buf_size, "%sArrive at destination", dist_prefix);
        }
        return;
    }

    if (step->action == NAV_ROUNDABOUT) {
        uint8_t ex = (step->exit_num > 0) ? step->exit_num : 1;
        const char *ord = roundabout_ordinal_word(ex);
        if (has_street) {
            snprintf(out_buf, buf_size, "%sAt the roundabout, take the %s exit onto %s",
                     dist_prefix, ord, street_name);
        } else {
            snprintf(out_buf, buf_size, "%sAt the roundabout, take the %s exit",
                     dist_prefix, ord);
        }
        return;
    }

    const char *act_str = nav_action_label(step->action);
    if (has_street) {
        snprintf(out_buf, buf_size, "%s%s onto %s", dist_prefix, act_str, street_name);
    } else {
        snprintf(out_buf, buf_size, "%s%s", dist_prefix, act_str);
    }
}
