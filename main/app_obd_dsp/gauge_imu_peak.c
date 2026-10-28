#include "gauge_imu_peak.h"
#include <math.h>
#include <string.h>

typedef struct { double x, y; } point_d_t;
/* Immutable normals: no lazy initialization, heap, or trigonometry per sample. */
static const point_d_t normals[GM_PEAK_DIRECTIONS] = {
    { 1, 0 },
    { 0.99518472667219693, 0.098017140329560604 },
    { 0.98078528040323043, 0.19509032201612825 },
    { 0.95694033573220882, 0.29028467725446233 },
    { 0.92387953251128674, 0.38268343236508978 },
    { 0.88192126434835505, 0.47139673682599764 },
    { 0.83146961230254524, 0.55557023301960218 },
    { 0.77301045336273699, 0.63439328416364549 },
    { 0.70710678118654757, 0.70710678118654746 },
    { 0.63439328416364549, 0.77301045336273699 },
    { 0.55557023301960229, 0.83146961230254524 },
    { 0.47139673682599781, 0.88192126434835494 },
    { 0.38268343236508984, 0.92387953251128674 },
    { 0.29028467725446233, 0.95694033573220882 },
    { 0.19509032201612833, 0.98078528040323043 },
    { 0.09801714032956077, 0.99518472667219682 },
    { 0, 1 },
    { -0.098017140329560645, 0.99518472667219693 },
    { -0.19509032201612819, 0.98078528040323043 },
    { -0.29028467725446216, 0.95694033573220894 },
    { -0.38268343236508973, 0.92387953251128674 },
    { -0.4713967368259977, 0.88192126434835505 },
    { -0.55557023301960196, 0.83146961230254535 },
    { -0.63439328416364538, 0.7730104533627371 },
    { -0.70710678118654746, 0.70710678118654757 },
    { -0.77301045336273699, 0.63439328416364549 },
    { -0.83146961230254535, 0.55557023301960218 },
    { -0.88192126434835494, 0.47139673682599786 },
    { -0.92387953251128674, 0.38268343236508989 },
    { -0.95694033573220882, 0.29028467725446239 },
    { -0.98078528040323043, 0.19509032201612861 },
    { -0.99518472667219682, 0.098017140329560826 },
    { -1, 0 },
    { -0.99518472667219693, -0.09801714032956059 },
    { -0.98078528040323043, -0.19509032201612836 },
    { -0.95694033573220894, -0.29028467725446211 },
    { -0.92387953251128685, -0.38268343236508967 },
    { -0.88192126434835505, -0.47139673682599764 },
    { -0.83146961230254546, -0.55557023301960196 },
    { -0.7730104533627371, -0.63439328416364527 },
    { -0.70710678118654768, -0.70710678118654746 },
    { -0.63439328416364582, -0.77301045336273666 },
    { -0.55557023301960218, -0.83146961230254524 },
    { -0.47139673682599792, -0.88192126434835494 },
    { -0.38268343236509034, -0.92387953251128652 },
    { -0.29028467725446244, -0.95694033573220882 },
    { -0.19509032201612866, -0.98078528040323032 },
    { -0.098017140329560451, -0.99518472667219693 },
    { 0, -1 },
    { 0.09801714032956009, -0.99518472667219693 },
    { 0.1950903220161283, -0.98078528040323043 },
    { 0.29028467725446205, -0.95694033573220894 },
    { 0.38268343236509, -0.92387953251128663 },
    { 0.47139673682599759, -0.88192126434835505 },
    { 0.55557023301960184, -0.83146961230254546 },
    { 0.6343932841636456, -0.77301045336273688 },
    { 0.70710678118654735, -0.70710678118654768 },
    { 0.77301045336273666, -0.63439328416364593 },
    { 0.83146961230254524, -0.55557023301960218 },
    { 0.88192126434835483, -0.47139673682599798 },
    { 0.92387953251128652, -0.38268343236509039 },
    { 0.95694033573220882, -0.2902846772544625 },
    { 0.98078528040323032, -0.19509032201612872 },
    { 0.99518472667219693, -0.098017140329560506 },
};

static float round_up(double value)
{
    float result = (float)value;
    return (double)result < value ? nextafterf(result, INFINITY) : result;
}

void gm_peak_clear(gm_peak_t *peak)
{
    if (peak) memset(peak, 0, sizeof(*peak));
}

void gm_peak_init(gm_peak_t *peak) { gm_peak_clear(peak); }

bool gm_peak_add(gm_peak_t *peak, float x_g, float y_g)
{
    if (!peak || !isfinite(x_g) || !isfinite(y_g) ||
        fabsf(x_g) > GM_PEAK_INPUT_LIMIT_G ||
        fabsf(y_g) > GM_PEAK_INPUT_LIMIT_G) return false;
    for (size_t i = 0; i < GM_PEAK_DIRECTIONS; ++i) {
        const double projection = normals[i].x * x_g + normals[i].y * y_g;
        if (projection > peak->support_g[i]) peak->support_g[i] = round_up(projection);
    }
    const float radius = round_up(hypot((double)x_g, (double)y_g));
    if (radius > peak->radius_g) peak->radius_g = radius;
    if (peak->sample_count != UINT32_MAX) ++peak->sample_count;
    return true;
}

static bool valid_state(const gm_peak_t *peak)
{
    if (!peak || !isfinite(peak->radius_g) || peak->radius_g < 0) return false;
    for (size_t i = 0; i < GM_PEAK_DIRECTIONS; ++i) {
        if (!isfinite(peak->support_g[i]) || peak->support_g[i] < 0) return false;
    }
    return true;
}

static bool append(point_d_t *points, size_t *count, point_d_t point)
{
    if (*count >= GM_PEAK_MAX_VERTICES) return false;
    points[(*count)++] = point;
    return true;
}

static double cross(point_d_t a, point_d_t b, point_d_t c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/* Float rounding can produce duplicate vertices and microscopic reflex edges
 * on a nearly degenerate envelope. Re-hull the rounded vertices so consumers
 * receive an actual convex CCW polygon, not those precision artifacts. */
static size_t rounded_hull(point_d_t *points, point_d_t *hull, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        points[i].x = (float)points[i].x;
        points[i].y = (float)points[i].y;
        const point_d_t value = points[i];
        size_t j = i;
        while (j && (points[j - 1].x > value.x ||
              (points[j - 1].x == value.x && points[j - 1].y > value.y))) {
            points[j] = points[j - 1]; --j;
        }
        points[j] = value;
    }
    size_t unique = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!unique || points[i].x != points[unique - 1].x ||
            points[i].y != points[unique - 1].y) points[unique++] = points[i];
    }
    if (unique == 1) { hull[0] = points[0]; return 1; }
    size_t n = 0;
    for (size_t i = 0; i < unique; ++i) {
        while (n >= 2 && cross(hull[n - 2], hull[n - 1], points[i]) <= 0) --n;
        if (!append(hull, &n, points[i])) return 0;
    }
    const size_t lower = n;
    for (size_t i = unique - 1; i > 0; --i) {
        while (n > lower && cross(hull[n - 2], hull[n - 1], points[i - 1]) <= 0) --n;
        if (!append(hull, &n, points[i - 1])) return 0;
    }
    return n - 1; /* Closing point equals first. */
}

size_t gm_peak_polygon(const gm_peak_t *peak, gm_peak_point_t *out, size_t capacity)
{
    if (!out || !valid_state(peak) || !peak->sample_count) return 0;
    if (peak->radius_g == 0) {
        if (!capacity) return 0;
        out[0] = (gm_peak_point_t){0, 0};
        return 1;
    }
    /* Padding dominates final float rounding. It depends on monotone radius,
     * so the mathematical intersection grows monotonically too. This is a
     * conservative 64-direction envelope, not an exact historical convex hull. */
    const double pad = 1e-6 * (1.0 + peak->radius_g);
    point_d_t scratch[2][GM_PEAK_MAX_VERTICES];
    point_d_t *src = scratch[0], *dst = scratch[1];
    const double left = -peak->support_g[32] - pad;
    const double right = peak->support_g[0] + pad;
    const double bottom = -peak->support_g[48] - pad;
    const double top = peak->support_g[16] + pad;
    src[0] = (point_d_t){left, bottom};
    src[1] = (point_d_t){right, bottom};
    src[2] = (point_d_t){right, top};
    src[3] = (point_d_t){left, top};
    size_t count = 4;
    for (size_t i = 0; i < GM_PEAK_DIRECTIONS; ++i) {
        const point_d_t normal = normals[i];
        const double limit = peak->support_g[i] + pad;
        size_t next_count = 0;
        point_d_t previous = src[count - 1];
        double previous_side = normal.x * previous.x + normal.y * previous.y - limit;
        for (size_t j = 0; j < count; ++j) {
            const point_d_t current = src[j];
            const double side = normal.x * current.x + normal.y * current.y - limit;
            if ((side <= 0) != (previous_side <= 0)) {
                const double fraction = previous_side / (previous_side - side);
                const point_d_t intersection = {
                    previous.x + fraction * (current.x - previous.x),
                    previous.y + fraction * (current.y - previous.y)
                };
                if (!append(dst, &next_count, intersection)) return 0;
            }
            if (side <= 0 && !append(dst, &next_count, current)) return 0;
            previous = current;
            previous_side = side;
        }
        if (!next_count) return 0;
        count = next_count;
        point_d_t *swap = src; src = dst; dst = swap;
    }
    count = rounded_hull(src, dst, count);
    if (!count || count > capacity) return 0;
    for (size_t i = 0; i < count; ++i) {
        out[i] = (gm_peak_point_t){(float)dst[i].x, (float)dst[i].y};
    }
    return count;
}

void gm_peak_snapshot(const gm_peak_t *peak, gm_peak_snapshot_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!valid_state(peak)) return;
    out->radius_g = peak->radius_g;
    out->sample_count = peak->sample_count;
    out->vertex_count = (uint8_t)gm_peak_polygon(peak, out->vertices, GM_PEAK_MAX_VERTICES);
}

bool gm_peak_contains(const gm_peak_t *peak, float x_g, float y_g, float tolerance_g)
{
    if (!valid_state(peak) || !peak->sample_count || !isfinite(x_g) || !isfinite(y_g) ||
        !isfinite(tolerance_g) || tolerance_g < 0) return false;
    for (size_t i = 0; i < GM_PEAK_DIRECTIONS; ++i) {
        if (normals[i].x * x_g + normals[i].y * y_g >
            (double)peak->support_g[i] + tolerance_g) return false;
    }
    return true;
}

float gm_peak_radius(const gm_peak_t *peak)
{
    return valid_state(peak) ? peak->radius_g : 0;
}
