/**
 * @file geocoder.c
 * @brief Forward & Reverse Geocoding Engine using Binary Map Files.
 *
 * Uses pure integer/fixed-point arithmetic and static buffers (zero malloc)
 * for direct portability between the desktop simulator and STM32 firmware.
 */

#include "geocoder.h"
#include <stdio.h>
#include <string.h>

/* Belgium 0.1° grid bounds for concentric ring tile search */
#define BELGIUM_GLAT_MIN 494
#define BELGIUM_GLAT_MAX 516
#define BELGIUM_GLON_MIN 25
#define BELGIUM_GLON_MAX 65

/*============================================================================
 * 1. Integer Math Helpers (Zero Floating-Point)
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

/**
 * @brief Compute planar distance in meters between two 1e7 coordinates.
 */
static uint32_t calc_dist_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    int64_t dlat = (int64_t)lat2_e7 - (int64_t)lat1_e7;
    int64_t dlon = (int64_t)lon2_e7 - (int64_t)lon1_e7;
    int64_t dy_m = (dlat * 111320LL) / 10000000LL;
    int64_t dx_m = (dlon * 70500LL)  / 10000000LL;
    return isqrt_u64((uint64_t)(dx_m * dx_m + dy_m * dy_m));
}

/**
 * @brief Compute shortest distance in meters from point P to segment A->B,
 *        plus the closest projected point on segment A->B.
 */
static uint32_t point_to_segment_dist_m(int32_t p_lat, int32_t p_lon,
                                        int32_t a_lat, int32_t a_lon,
                                        int32_t b_lat, int32_t b_lon,
                                        GeoCoord *out_closest) {
    /* Convert deltas relative to A into millimeters for high precision integer projection */
    int64_t px_mm = ((int64_t)(p_lon - a_lon) * 70500LL)  / 10000LL;
    int64_t py_mm = ((int64_t)(p_lat - a_lat) * 111320LL) / 10000LL;

    int64_t bx_mm = ((int64_t)(b_lon - a_lon) * 70500LL)  / 10000LL;
    int64_t by_mm = ((int64_t)(b_lat - a_lat) * 111320LL) / 10000LL;

    int64_t len_sq = bx_mm * bx_mm + by_mm * by_mm;
    int64_t proj_x_mm = 0;
    int64_t proj_y_mm = 0;

    if (len_sq <= 0) {
        if (out_closest) {
            out_closest->lat_e7 = a_lat;
            out_closest->lon_e7 = a_lon;
        }
    } else {
        int64_t dot = px_mm * bx_mm + py_mm * by_mm;
        if (dot <= 0) {
            if (out_closest) {
                out_closest->lat_e7 = a_lat;
                out_closest->lon_e7 = a_lon;
            }
        } else if (dot >= len_sq) {
            proj_x_mm = bx_mm;
            proj_y_mm = by_mm;
            if (out_closest) {
                out_closest->lat_e7 = b_lat;
                out_closest->lon_e7 = b_lon;
            }
        } else {
            /* t in [0 .. 1024] fixed-point */
            int64_t t_q10 = (dot << 10) / len_sq;
            proj_x_mm = (bx_mm * t_q10) >> 10;
            proj_y_mm = (by_mm * t_q10) >> 10;
            if (out_closest) {
                out_closest->lat_e7 = a_lat + (int32_t)(((int64_t)(b_lat - a_lat) * t_q10) >> 10);
                out_closest->lon_e7 = a_lon + (int32_t)(((int64_t)(b_lon - a_lon) * t_q10) >> 10);
            }
        }
    }

    int64_t dx_m = (px_mm - proj_x_mm) / 1000LL;
    int64_t dy_m = (py_mm - proj_y_mm) / 1000LL;
    return isqrt_u64((uint64_t)(dx_m * dx_m + dy_m * dy_m));
}

/*============================================================================
 * 2. UTF-8 Normalization (Matches map_binaries_gen.py normalize_name)
 *============================================================================*/

/**
 * @brief Decode one UTF-8 codepoint and advance *p.
 */
static uint32_t next_utf8_codepoint(const unsigned char **p) {
    const unsigned char *s = *p;
    if (*s == 0) return 0;

    if (s[0] < 0x80) {
        *p = s + 1;
        return s[0];
    } else if ((s[0] & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
        *p = s + 2;
        return ((uint32_t)(s[0] & 0x1F) << 6) | (uint32_t)(s[1] & 0x3F);
    } else if ((s[0] & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80) {
        *p = s + 3;
        return ((uint32_t)(s[0] & 0x0F) << 12) |
               ((uint32_t)(s[1] & 0x3F) << 6)  |
               (uint32_t)(s[2] & 0x3F);
    } else if ((s[0] & 0xF8) == 0xF0 && (s[1] & 0xC0) == 0x80 &&
               (s[2] & 0xC0) == 0x80 && (s[3] & 0xC0) == 0x80) {
        *p = s + 4;
        return ((uint32_t)(s[0] & 0x07) << 18) |
               ((uint32_t)(s[1] & 0x3F) << 12) |
               ((uint32_t)(s[2] & 0x3F) << 6)  |
               (uint32_t)(s[3] & 0x3F);
    }

    /* Invalid byte fallback */
    *p = s + 1;
    return s[0];
}

/**
 * @brief Map a Unicode codepoint to its NFKD base ASCII lowercase character.
 *        Returns 0 if the codepoint is a combining mark or non-alphanumeric symbol.
 */
static char unicode_to_base_ascii(uint32_t cp) {
    /* Skip Unicode combining diacritical marks (U+0300 .. U+036F) */
    if (cp >= 0x0300 && cp <= 0x036F) {
        return 0;
    }

    if (cp >= 'A' && cp <= 'Z') return (char)(cp - 'A' + 'a');
    if (cp >= 'a' && cp <= 'z') return (char)cp;
    if (cp >= '0' && cp <= '9') return (char)cp;
    if (cp == ' ' || cp == '\t' || cp == '\n' || cp == '\r') return ' ';
    if (cp == '-') return '-';

    /* Latin-1 Supplement (U+00C0 .. U+00FF) */
    if (cp >= 0x00C0 && cp <= 0x00FF) {
        if (cp >= 0x00E0) cp -= 0x20; /* Fold lowercase Latin-1 to uppercase range */
        if (cp >= 0x00C0 && cp <= 0x00C5) return 'a'; /* À Á Â Ã Ä Å */
        if (cp == 0x00C7) return 'c';                 /* Ç */
        if (cp >= 0x00C8 && cp <= 0x00CB) return 'e'; /* È É Ê Ë */
        if (cp >= 0x00CC && cp <= 0x00CF) return 'i'; /* Ì Í Î Ï */
        if (cp == 0x00D1) return 'n';                 /* Ñ */
        if (cp >= 0x00D2 && cp <= 0x00D6) return 'o'; /* Ò Ó Ô Õ Ö */
        if (cp >= 0x00D9 && cp <= 0x00DC) return 'u'; /* Ù Ú Û Ü */
        if (cp == 0x00DD || cp == 0x00DF) return 'y'; /* Ý / Ÿ */
    }

    /* Selected Latin Extended-A characters common in French/Dutch/German names */
    switch (cp) {
        case 0x0100: case 0x0101: case 0x0102: case 0x0103: case 0x0104: case 0x0105: return 'a';
        case 0x0106: case 0x0107: case 0x0108: case 0x0109: case 0x010C: case 0x010D: return 'c';
        case 0x010E: case 0x010F: return 'd';
        case 0x0112: case 0x0113: case 0x0116: case 0x0117: case 0x0118: case 0x0119: case 0x011A: case 0x011B: return 'e';
        case 0x011E: case 0x011F: return 'g';
        case 0x012A: case 0x012B: case 0x012E: case 0x012F: return 'i';
        case 0x0141: case 0x0142: return 'l';
        case 0x0143: case 0x0144: case 0x0147: case 0x0148: return 'n';
        case 0x014C: case 0x014D: case 0x0150: case 0x0151: return 'o';
        case 0x0154: case 0x0155: case 0x0158: case 0x0159: return 'r';
        case 0x015A: case 0x015B: case 0x015E: case 0x015F: case 0x0160: case 0x0161: return 's';
        case 0x0162: case 0x0163: case 0x0164: case 0x0165: return 't';
        case 0x016A: case 0x016B: case 0x016E: case 0x016F: case 0x0170: case 0x0171: return 'u';
        case 0x0178: return 'y';
        case 0x0179: case 0x017A: case 0x017B: case 0x017C: case 0x017D: case 0x017E: return 'z';
        default: return 0;
    }
}

void geocoder_normalize_name(const char *input, char *out_buf, size_t out_size) {
    if (!out_buf || out_size == 0) return;
    out_buf[0] = '\0';
    if (!input) return;

    const unsigned char *p = (const unsigned char *)input;
    size_t idx = 0;
    bool prev_space = true; /* Trim leading whitespace */

    while (*p != '\0' && idx + 1 < out_size) {
        uint32_t cp = next_utf8_codepoint(&p);
        char c = unicode_to_base_ascii(cp);
        if (c == 0) continue;

        if (c == ' ') {
            if (!prev_space) {
                out_buf[idx++] = ' ';
                prev_space = true;
            }
        } else {
            out_buf[idx++] = c;
            prev_space = false;
        }
    }

    /* Strip trailing space */
    if (idx > 0 && out_buf[idx - 1] == ' ') {
        idx--;
    }
    out_buf[idx] = '\0';
}

/*============================================================================
 * 3. Initialization & String Table Readers
 *============================================================================*/

static bool read_null_term_string(FILE *f_str, uint32_t offset, char *out_buf, size_t out_size) {
    if (!f_str || !out_buf || out_size == 0) return false;
    if (fseek(f_str, (long)offset, SEEK_SET) != 0) {
        out_buf[0] = '\0';
        return false;
    }
    size_t n = fread(out_buf, 1, out_size - 1, f_str);
    out_buf[n] = '\0';
    return (n > 0);
}

bool geocoder_init(Geocoder *geo, TileGraphCache *cache, const char *base_dir) {
    if (!geo) return false;
    memset(geo, 0, sizeof(Geocoder));
    geo->graph_cache = cache;

    const char *dir = (base_dir && base_dir[0] != '\0') ? base_dir : "../binary_map_data";

    snprintf(geo->names_idx_path, sizeof(geo->names_idx_path), "%s/names.idx", dir);
    snprintf(geo->names_str_path, sizeof(geo->names_str_path), "%s/names.str", dir);
    snprintf(geo->towns_idx_path, sizeof(geo->towns_idx_path), "%s/towns.idx", dir);
    snprintf(geo->towns_str_path, sizeof(geo->towns_str_path), "%s/towns.str", dir);

    FILE *f_nidx = fopen(geo->names_idx_path, "rb");
    if (!f_nidx) return false;
    fseek(f_nidx, 0, SEEK_END);
    long n_size = ftell(f_nidx);
    fclose(f_nidx);
    if (n_size <= 0 || (n_size % (long)sizeof(NameIndexRecord)) != 0) {
        return false;
    }
    geo->street_count = (uint32_t)(n_size / (long)sizeof(NameIndexRecord));

    FILE *f_tidx = fopen(geo->towns_idx_path, "rb");
    if (f_tidx) {
        fseek(f_tidx, 0, SEEK_END);
        long t_size = ftell(f_tidx);
        fclose(f_tidx);
        if (t_size > 0 && (t_size % (long)sizeof(TownIndexRecord)) == 0) {
            geo->town_count = (uint32_t)(t_size / (long)sizeof(TownIndexRecord));
        }
    }

    return true;
}

bool geocoder_get_street_name(const Geocoder *geo, uint32_t street_id, char *out_name, size_t out_size) {
    if (!geo || !out_name || out_size == 0) return false;
    out_name[0] = '\0';

    if (street_id == 0 || street_id > geo->street_count) {
        return false;
    }

    FILE *f_idx = fopen(geo->names_idx_path, "rb");
    if (!f_idx) return false;

    NameIndexRecord rec;
    uint32_t idx = street_id - 1U;
    if (fseek(f_idx, (long)(idx * sizeof(NameIndexRecord)), SEEK_SET) != 0 ||
        fread(&rec, sizeof(NameIndexRecord), 1, f_idx) != 1) {
        fclose(f_idx);
        return false;
    }
    fclose(f_idx);

    FILE *f_str = fopen(geo->names_str_path, "rb");
    if (!f_str) return false;
    bool ok = read_null_term_string(f_str, rec.str_offset, out_name, out_size);
    fclose(f_str);
    return ok;
}

/*============================================================================
 * 4. Town Lookup & Reverse Town Matching
 *============================================================================*/

bool geocoder_find_nearest_town(const Geocoder *geo, GeoCoord coord,
                                char *out_name, size_t out_size,
                                GeoCoord *out_coord, uint32_t *out_dist_m) {
    if (!geo || geo->town_count == 0) return false;

    FILE *f_idx = fopen(geo->towns_idx_path, "rb");
    if (!f_idx) return false;

    TownIndexRecord best_rec = {0};
    uint32_t best_dist = 0xFFFFFFFFU;
    bool found = false;

    TownIndexRecord buf[64];
    uint32_t remaining = geo->town_count;
    while (remaining > 0) {
        uint32_t chunk = (remaining > 64U) ? 64U : remaining;
        if (fread(buf, sizeof(TownIndexRecord), chunk, f_idx) != chunk) break;
        for (uint32_t i = 0; i < chunk; ++i) {
            uint32_t d = calc_dist_m(coord.lat_e7, coord.lon_e7, buf[i].lat_scaled, buf[i].lon_scaled);
            if (d < best_dist) {
                best_dist = d;
                best_rec = buf[i];
                found = true;
            }
        }
        remaining -= chunk;
    }
    fclose(f_idx);

    if (!found) return false;

    if (out_coord) {
        out_coord->lat_e7 = best_rec.lat_scaled;
        out_coord->lon_e7 = best_rec.lon_scaled;
    }
    if (out_dist_m) {
        *out_dist_m = best_dist;
    }
    if (out_name && out_size > 0) {
        FILE *f_str = fopen(geo->towns_str_path, "rb");
        if (f_str) {
            read_null_term_string(f_str, (uint32_t)best_rec.str_offset, out_name, out_size);
            fclose(f_str);
        } else {
            out_name[0] = '\0';
        }
    }
    return true;
}

/**
 * @brief Find a town by name in towns.idx / towns.str.
 *        Prefers exact normalized match, then prefix match, then substring match.
 *        If multiple towns match at the same tier, picks the one closest to ref_coord.
 */
static bool find_town_by_name(const Geocoder *geo, const char *town_query, GeoCoord ref_coord,
                              GeoCoord *out_town_coord, char *out_town_name, size_t name_size) {
    if (!geo || geo->town_count == 0 || !town_query) return false;

    char norm_query[GEOCODER_NAME_MAX];
    geocoder_normalize_name(town_query, norm_query, sizeof(norm_query));
    if (norm_query[0] == '\0') return false;
    size_t q_len = strlen(norm_query);

    FILE *f_idx = fopen(geo->towns_idx_path, "rb");
    FILE *f_str = fopen(geo->towns_str_path, "rb");
    if (!f_idx || !f_str) {
        if (f_idx) fclose(f_idx);
        if (f_str) fclose(f_str);
        return false;
    }

    int best_tier = 99; /* 1 = exact, 2 = prefix, 3 = substring */
    uint32_t best_dist = 0xFFFFFFFFU;
    TownIndexRecord best_rec;
    char best_raw[GEOCODER_NAME_MAX] = {0};

    for (uint32_t i = 0; i < geo->town_count; ++i) {
        TownIndexRecord rec;
        if (fread(&rec, sizeof(TownIndexRecord), 1, f_idx) != 1) break;

        char raw_name[GEOCODER_NAME_MAX];
        if (!read_null_term_string(f_str, (uint32_t)rec.str_offset, raw_name, sizeof(raw_name))) {
            continue;
        }

        char norm_town[GEOCODER_NAME_MAX];
        geocoder_normalize_name(raw_name, norm_town, sizeof(norm_town));

        int tier = 99;
        if (strcmp(norm_town, norm_query) == 0) {
            tier = 1;
        } else if (strncmp(norm_town, norm_query, q_len) == 0) {
            tier = 2;
        } else if (strstr(norm_town, norm_query) != NULL) {
            tier = 3;
        }

        if (tier < 99) {
            uint32_t d = calc_dist_m(ref_coord.lat_e7, ref_coord.lon_e7, rec.lat_scaled, rec.lon_scaled);
            if (tier < best_tier || (tier == best_tier && d < best_dist)) {
                best_tier = tier;
                best_dist = d;
                best_rec = rec;
                snprintf(best_raw, sizeof(best_raw), "%s", raw_name);
            }
        }
    }

    fclose(f_idx);
    fclose(f_str);

    if (best_tier == 99) return false;

    if (out_town_coord) {
        out_town_coord->lat_e7 = best_rec.lat_scaled;
        out_town_coord->lon_e7 = best_rec.lon_scaled;
    }
    if (out_town_name && name_size > 0) {
        snprintf(out_town_name, name_size, "%s", best_raw);
    }
    return true;
}

/*============================================================================
 * 5. Forward Geocoding: Street Name -> Coordinate
 *============================================================================*/

/**
 * @brief Collect candidate 1-based street_ids matching norm_street_query.
 *        Step 1: O(log N) binary search in names.idx for exact or prefix matches.
 *        Step 2: Substring fallback scan if neither exact nor prefix matched.
 */
static int32_t find_candidate_street_ids(const Geocoder *geo, const char *norm_query,
                                         uint32_t *out_ids, int32_t max_ids) {
    if (!geo || geo->street_count == 0 || !norm_query || norm_query[0] == '\0') return 0;

    FILE *f_idx = fopen(geo->names_idx_path, "rb");
    FILE *f_str = fopen(geo->names_str_path, "rb");
    if (!f_idx || !f_str) {
        if (f_idx) fclose(f_idx);
        if (f_str) fclose(f_str);
        return 0;
    }

    size_t q_len = strlen(norm_query);
    int32_t count = 0;

    /* Binary search for lower bound of norm_query in names.idx */
    int32_t lo = 0;
    int32_t hi = (int32_t)geo->street_count - 1;
    int32_t lower_bound = (int32_t)geo->street_count;

    while (lo <= hi) {
        int32_t mid = lo + ((hi - lo) >> 1);
        NameIndexRecord rec;
        fseek(f_idx, (long)mid * (long)sizeof(NameIndexRecord), SEEK_SET);
        if (fread(&rec, sizeof(NameIndexRecord), 1, f_idx) != 1) break;

        char raw_name[GEOCODER_NAME_MAX];
        read_null_term_string(f_str, rec.str_offset, raw_name, sizeof(raw_name));

        char norm_name[GEOCODER_NAME_MAX];
        geocoder_normalize_name(raw_name, norm_name, sizeof(norm_name));

        int cmp = strcmp(norm_name, norm_query);
        if (cmp >= 0) {
            lower_bound = mid;
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }

    /* Check exact match first, then prefix matches starting at lower_bound */
    if (lower_bound < (int32_t)geo->street_count) {
        for (int32_t idx = lower_bound; idx < (int32_t)geo->street_count && count < max_ids; ++idx) {
            NameIndexRecord rec;
            fseek(f_idx, (long)idx * (long)sizeof(NameIndexRecord), SEEK_SET);
            if (fread(&rec, sizeof(NameIndexRecord), 1, f_idx) != 1) break;

            char raw_name[GEOCODER_NAME_MAX];
            read_null_term_string(f_str, rec.str_offset, raw_name, sizeof(raw_name));

            char norm_name[GEOCODER_NAME_MAX];
            geocoder_normalize_name(raw_name, norm_name, sizeof(norm_name));

            if (strcmp(norm_name, norm_query) == 0) {
                /* Exact match gets top priority */
                out_ids[count++] = (uint32_t)(idx + 1);
                break;
            } else if (strncmp(norm_name, norm_query, q_len) == 0) {
                out_ids[count++] = (uint32_t)(idx + 1);
            } else {
                break; /* Past alphabetical prefix range */
            }
        }
    }

    /* Fallback: If no exact or prefix match was found, scan for substring match */
    if (count == 0) {
        fseek(f_idx, 0, SEEK_SET);
        NameIndexRecord idx_buf[128];
        uint32_t global_idx = 0;
        while (global_idx < geo->street_count && count < max_ids) {
            uint32_t chunk = geo->street_count - global_idx;
            if (chunk > 128U) chunk = 128U;
            if (fread(idx_buf, sizeof(NameIndexRecord), chunk, f_idx) != chunk) break;

            for (uint32_t i = 0; i < chunk && count < max_ids; ++i) {
                char raw_name[GEOCODER_NAME_MAX];
                read_null_term_string(f_str, idx_buf[i].str_offset, raw_name, sizeof(raw_name));

                char norm_name[GEOCODER_NAME_MAX];
                geocoder_normalize_name(raw_name, norm_name, sizeof(norm_name));

                if (strstr(norm_name, norm_query) != NULL) {
                    out_ids[count++] = global_idx + i + 1U;
                }
            }
            global_idx += chunk;
        }
    }

    fclose(f_idx);
    fclose(f_str);
    return count;
}

/**
 * @brief Search a single local tile (glat, glon) for edges matching any of candidate_ids,
 *        updating best_dist_m, best_coord, and best_street_id relative to search_center.
 */
static bool search_tile_for_streets(Geocoder *geo, int32_t glat, int32_t glon,
                                    const uint32_t *candidate_ids, int32_t cand_count,
                                    GeoCoord search_center,
                                    uint32_t *io_best_dist_m,
                                    GeoCoord *io_best_coord,
                                    uint32_t *io_best_street_id) {
    if (!geo || !geo->graph_cache || cand_count <= 0) return false;

    /* 1. Check if this tile is already loaded in TileGraphCache RAM */
    for (int32_t s = 0; s < geo->graph_cache->local_capacity; ++s) {
        TileGraph *slot = &geo->graph_cache->local_slots[s];
        if (slot->loaded && slot->grid_lat == glat && slot->grid_lon == glon) {
            bool hit = false;
            for (uint32_t u_idx = 0; u_idx < slot->node_count; ++u_idx) {
                const NodeRecord *u = &slot->nodes[u_idx];
                uint32_t end_e = u->first_edge + u->edge_count;
                if (end_e > slot->edge_count) end_e = slot->edge_count;

                for (uint32_t e_idx = u->first_edge; e_idx < end_e; ++e_idx) {
                    uint32_t sid = slot->edges[e_idx].street_id;
                    if (sid == 0) continue;

                    for (int32_t c = 0; c < cand_count; ++c) {
                        if (sid == candidate_ids[c]) {
                            uint32_t v_idx = slot->edges[e_idx].target_idx;
                            if (v_idx < slot->node_count) {
                                const NodeRecord *v = &slot->nodes[v_idx];
                                int32_t mlat = (u->lat_scaled + v->lat_scaled) / 2;
                                int32_t mlon = (u->lon_scaled + v->lon_scaled) / 2;
                                uint32_t d = calc_dist_m(search_center.lat_e7, search_center.lon_e7, mlat, mlon);
                                if (d < *io_best_dist_m) {
                                    *io_best_dist_m    = d;
                                    io_best_coord->lat_e7 = mlat;
                                    io_best_coord->lon_e7 = mlon;
                                    *io_best_street_id = sid;
                                    hit = true;
                                }
                            }
                            break;
                        }
                    }
                }
            }
            return hit;
        }
    }

    /* 2. Otherwise stream local_XXX_YYY_nodes.bin & edges.bin from disk without evicting cache slots */
    char edges_path[GEOCODER_PATH_MAX];
    char nodes_path[GEOCODER_PATH_MAX];
    snprintf(edges_path, sizeof(edges_path), "%s/local_%03ld_%03ld_edges.bin",
             geo->graph_cache->base_dir, (long)glat, (long)glon);
    snprintf(nodes_path, sizeof(nodes_path), "%s/local_%03ld_%03ld_nodes.bin",
             geo->graph_cache->base_dir, (long)glat, (long)glon);

    FILE *f_edges = fopen(edges_path, "rb");
    if (!f_edges) return false;
    FILE *f_nodes = fopen(nodes_path, "rb");
    if (!f_nodes) {
        fclose(f_edges);
        return false;
    }

    bool hit = false;
    EdgeRecord e_buf[128];
    size_t n_read;
    while ((n_read = fread(e_buf, sizeof(EdgeRecord), 128, f_edges)) > 0) {
        for (size_t i = 0; i < n_read; ++i) {
            uint32_t sid = e_buf[i].street_id;
            if (sid == 0) continue;

            for (int32_t c = 0; c < cand_count; ++c) {
                if (sid == candidate_ids[c]) {
                    uint32_t v_idx = e_buf[i].target_idx;
                    NodeRecord node_v;
                    if (fseek(f_nodes, (long)(v_idx * sizeof(NodeRecord)), SEEK_SET) == 0 &&
                        fread(&node_v, sizeof(NodeRecord), 1, f_nodes) == 1) {
                        uint32_t d = calc_dist_m(search_center.lat_e7, search_center.lon_e7,
                                                 node_v.lat_scaled, node_v.lon_scaled);
                        if (d < *io_best_dist_m) {
                            *io_best_dist_m = d;
                            io_best_coord->lat_e7 = node_v.lat_scaled;
                            io_best_coord->lon_e7 = node_v.lon_scaled;
                            *io_best_street_id = sid;
                            hit = true;
                        }
                    }
                    break;
                }
            }
        }
    }

    fclose(f_nodes);
    fclose(f_edges);
    return hit;
}

bool geocoder_street_to_coord(Geocoder *geo, const char *query, GeoCoord ref_coord,
                              GeocoderResult *out_result) {
    if (!out_result) return false;
    memset(out_result, 0, sizeof(GeocoderResult));
    if (!geo || !geo->graph_cache || !query) return false;

    /* 1. Split query on ',' into [street name] and optional [town] */
    char street_raw[GEOCODER_NAME_MAX] = {0};
    char town_raw[GEOCODER_NAME_MAX]   = {0};
    bool has_town_query = false;

    const char *comma = strchr(query, ',');
    if (comma) {
        size_t s_len = (size_t)(comma - query);
        if (s_len >= sizeof(street_raw)) s_len = sizeof(street_raw) - 1;
        memcpy(street_raw, query, s_len);
        street_raw[s_len] = '\0';

        strncpy(town_raw, comma + 1, sizeof(town_raw) - 1);
        town_raw[sizeof(town_raw) - 1] = '\0';

        char norm_town_check[GEOCODER_NAME_MAX];
        geocoder_normalize_name(town_raw, norm_town_check, sizeof(norm_town_check));
        if (norm_town_check[0] != '\0') {
            has_town_query = true;
        }
    } else {
        strncpy(street_raw, query, sizeof(street_raw) - 1);
        street_raw[sizeof(street_raw) - 1] = '\0';
    }

    char norm_street[GEOCODER_NAME_MAX];
    geocoder_normalize_name(street_raw, norm_street, sizeof(norm_street));
    if (norm_street[0] == '\0') return false;

    /* 2. Resolve town center if a town was specified, otherwise use ref_coord */
    GeoCoord search_center = ref_coord;
    char matched_town_name[GEOCODER_NAME_MAX] = {0};

    if (has_town_query) {
        if (!find_town_by_name(geo, town_raw, ref_coord, &search_center,
                               matched_town_name, sizeof(matched_town_name))) {
            return false;
        }
    }

    /* 3. Find candidate street_ids from names.idx / names.str */
    uint32_t candidate_ids[GEOCODER_MAX_CANDIDATES];
    int32_t cand_count = find_candidate_street_ids(geo, norm_street,
                                                   candidate_ids, GEOCODER_MAX_CANDIDATES);
    if (cand_count <= 0) return false;

    /* 4. Search outward in concentric grid rings around search_center */
    int32_t center_glat = 0, center_glon = 0;
    geo_coord_to_grid(search_center.lat_e7, search_center.lon_e7, &center_glat, &center_glon);

    /* When a town is specified, search up to radius 2 (~20 km around town center);
     * when no town is specified, search outward across Belgium until the closest match is guaranteed. */
    int32_t max_radius = has_town_query ? 2 : 30;

    uint32_t best_dist_m = 0xFFFFFFFFU;
    GeoCoord best_coord  = {0, 0};
    uint32_t best_sid    = 0;

    for (int32_t r = 0; r <= max_radius; ++r) {
        for (int32_t dlat = -r; dlat <= r; ++dlat) {
            for (int32_t dlon = -r; dlon <= r; ++dlon) {
                /* Only visit perimeter cells of ring r */
                if (r > 0 && dlat > -r && dlat < r && dlon > -r && dlon < r) {
                    continue;
                }

                int32_t glat = center_glat + dlat;
                int32_t glon = center_glon + dlon;
                if (glat < BELGIUM_GLAT_MIN || glat > BELGIUM_GLAT_MAX ||
                    glon < BELGIUM_GLON_MIN || glon > BELGIUM_GLON_MAX) {
                    continue;
                }

                search_tile_for_streets(geo, glat, glon, candidate_ids, cand_count,
                                        search_center, &best_dist_m, &best_coord, &best_sid);
            }
        }

        /* Early exit: once we have a match and have checked at least 1 ring beyond it,
         * and the minimum possible distance to ring (r + 1) exceeds best_dist_m */
        if (best_sid != 0 && r >= 1) {
            uint32_t min_next_ring_m = (uint32_t)r * 7000U;
            if (best_dist_m <= min_next_ring_m) {
                break;
            }
        }
    }

    if (best_sid == 0) return false;

    /* 5. Populate output result */
    out_result->found      = true;
    out_result->coord      = best_coord;
    out_result->street_id  = best_sid;
    out_result->distance_m = calc_dist_m(ref_coord.lat_e7, ref_coord.lon_e7,
                                         best_coord.lat_e7, best_coord.lon_e7);

    geocoder_get_street_name(geo, best_sid, out_result->street_name, sizeof(out_result->street_name));

    if (has_town_query && matched_town_name[0] != '\0') {
        snprintf(out_result->town_name, sizeof(out_result->town_name), "%s", matched_town_name);
    } else {
        geocoder_find_nearest_town(geo, best_coord, out_result->town_name,
                                   sizeof(out_result->town_name), NULL, NULL);
    }

    return true;
}

/*============================================================================
 * 6. Reverse Geocoding: Coordinate -> Street Name & Town
 *============================================================================*/

static void check_tile_reverse(const TileGraph *tile, GeoCoord coord,
                               uint32_t *io_best_dist_m, GeoCoord *io_best_snapped,
                               uint32_t *io_best_street_id) {
    if (!tile || !tile->loaded) return;

    /* Pre-filter bounding box (~1.1 km around coord) */
    const int32_t margin_e7 = 100000; /* 0.01 degrees */
    int32_t min_lat = coord.lat_e7 - margin_e7;
    int32_t max_lat = coord.lat_e7 + margin_e7;
    int32_t min_lon = coord.lon_e7 - margin_e7;
    int32_t max_lon = coord.lon_e7 + margin_e7;

    FILE *f_geom = fopen(tile->geom_path, "rb");

    for (uint32_t u_idx = 0; u_idx < tile->node_count; ++u_idx) {
        const NodeRecord *u = &tile->nodes[u_idx];
        if (u->lat_scaled < min_lat || u->lat_scaled > max_lat ||
            u->lon_scaled < min_lon || u->lon_scaled > max_lon) {
            continue;
        }

        uint32_t first_e = u->first_edge;
        uint32_t end_e   = first_e + u->edge_count;
        if (end_e > tile->edge_count) end_e = tile->edge_count;

        for (uint32_t e_idx = first_e; e_idx < end_e; ++e_idx) {
            const EdgeRecord *e = &tile->edges[e_idx];
            if (e->street_id == 0 || e->target_idx >= tile->node_count) {
                continue;
            }

            const NodeRecord *v = &tile->nodes[e->target_idx];

            /* Check intermediate curve segments if geometry is available */
            if (f_geom && e->geom_count > 0) {
                if (fseek(f_geom, (long)e->geom_offset, SEEK_SET) == 0) {
                    int32_t prev_lat = u->lat_scaled;
                    int32_t prev_lon = u->lon_scaled;
                    uint16_t rem = e->geom_count;
                    GeomPoint g_buf[32];

                    while (rem > 0) {
                        uint16_t chunk = (rem > 32U) ? 32U : rem;
                        if (fread(g_buf, sizeof(GeomPoint), chunk, f_geom) != chunk) break;
                        for (uint16_t k = 0; k < chunk; ++k) {
                            GeoCoord snapped;
                            uint32_t d = point_to_segment_dist_m(
                                coord.lat_e7, coord.lon_e7,
                                prev_lat, prev_lon,
                                g_buf[k].lat_scaled, g_buf[k].lon_scaled,
                                &snapped
                            );
                            if (d < *io_best_dist_m) {
                                *io_best_dist_m     = d;
                                *io_best_snapped    = snapped;
                                *io_best_street_id  = e->street_id;
                            }
                            prev_lat = g_buf[k].lat_scaled;
                            prev_lon = g_buf[k].lon_scaled;
                        }
                        rem -= chunk;
                    }

                    GeoCoord snapped;
                    uint32_t d = point_to_segment_dist_m(
                        coord.lat_e7, coord.lon_e7,
                        prev_lat, prev_lon,
                        v->lat_scaled, v->lon_scaled,
                        &snapped
                    );
                    if (d < *io_best_dist_m) {
                        *io_best_dist_m     = d;
                        *io_best_snapped    = snapped;
                        *io_best_street_id  = e->street_id;
                    }
                    continue;
                }
            }

            /* Straight segment fallback u -> v */
            GeoCoord snapped;
            uint32_t d = point_to_segment_dist_m(
                coord.lat_e7, coord.lon_e7,
                u->lat_scaled, u->lon_scaled,
                v->lat_scaled, v->lon_scaled,
                &snapped
            );
            if (d < *io_best_dist_m) {
                *io_best_dist_m     = d;
                *io_best_snapped    = snapped;
                *io_best_street_id  = e->street_id;
            }
        }
    }

    if (f_geom) {
        fclose(f_geom);
    }
}

bool geocoder_coord_to_street(Geocoder *geo, GeoCoord coord, GeocoderResult *out_result) {
    if (!out_result) return false;
    memset(out_result, 0, sizeof(GeocoderResult));
    if (!geo || !geo->graph_cache) return false;

    int32_t glat = 0, glon = 0;
    geo_coord_to_grid(coord.lat_e7, coord.lon_e7, &glat, &glon);

    uint32_t best_dist_m    = 0xFFFFFFFFU;
    GeoCoord best_snapped   = coord;
    uint32_t best_street_id = 0;

    /* Check primary tile first */
    TileGraph *tile = tile_graph_cache_get_tile(geo->graph_cache, glat, glon);
    if (tile) {
        check_tile_reverse(tile, coord, &best_dist_m, &best_snapped, &best_street_id);
    }

    /* If no street found within 100m (e.g. near tile boundary), check adjacent tiles */
    if (best_dist_m > 100U) {
        for (int32_t dlat = -1; dlat <= 1; ++dlat) {
            for (int32_t dlon = -1; dlon <= 1; ++dlon) {
                if (dlat == 0 && dlon == 0) continue;
                TileGraph *adj = tile_graph_cache_get_tile(geo->graph_cache, glat + dlat, glon + dlon);
                if (adj) {
                    check_tile_reverse(adj, coord, &best_dist_m, &best_snapped, &best_street_id);
                }
            }
        }
    }

    if (best_street_id == 0) return false;

    out_result->found      = true;
    out_result->coord      = best_snapped;
    out_result->street_id  = best_street_id;
    out_result->distance_m = best_dist_m;

    geocoder_get_street_name(geo, best_street_id,
                             out_result->street_name, sizeof(out_result->street_name));
    geocoder_find_nearest_town(geo, best_snapped,
                               out_result->town_name, sizeof(out_result->town_name), NULL, NULL);

    return true;
}
