/*
 * Unit tests for PEP forward-progress helper functions (TR-10-13 Section 18).
 *
 * Build:
 *   cc -o test_pep_forward test_pep_forward.c && ./test_pep_forward
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/* ---- Replicate the static helpers from ipmx_usb_common.c ---- */

static int ctr_is_forward(uint64_t incoming, uint64_t last)
{
    uint64_t delta = incoming - last;
    return (delta > 0) && (delta < (UINT64_C(1) << 63));
}

static int kv_is_forward(uint32_t incoming, uint32_t last)
{
    uint32_t delta;
    if (incoming == last)
        return 1;
    delta = incoming - last;
    return (delta > 0) && (delta <= (UINT32_C(1) << 31));
}

/* ---- Test harness ---- */

static int pass_count = 0;
static int fail_count = 0;

#define CHECK(expr, label) do { \
    if (expr) { pass_count++; } \
    else { fail_count++; fprintf(stderr, "FAIL: %s\n", label); } \
} while (0)

static void test_ctr_basic(void)
{
    CHECK( ctr_is_forward(1, 0),   "ctr: 1 > 0");
    CHECK( ctr_is_forward(100, 1), "ctr: 100 > 1");
    CHECK(!ctr_is_forward(0, 0),   "ctr: 0 == 0 (not forward)");
    CHECK(!ctr_is_forward(0, 1),   "ctr: 0 < 1 (backward)");
    CHECK(!ctr_is_forward(5, 10),  "ctr: 5 < 10 (backward)");
}

static void test_ctr_gaps(void)
{
    CHECK(ctr_is_forward(10, 1),           "ctr: gap of 9 is fine");
    CHECK(ctr_is_forward(1000000, 1),      "ctr: large gap is fine");
    CHECK(ctr_is_forward(UINT64_C(1) << 62, 0), "ctr: half-range forward ok");
}

static void test_ctr_wrap(void)
{
    uint64_t max = UINT64_MAX;
    CHECK( ctr_is_forward(0, max),   "ctr: 0 wraps past MAX");
    CHECK( ctr_is_forward(1, max),   "ctr: 1 wraps past MAX");
    CHECK( ctr_is_forward(max, max - 1), "ctr: MAX > MAX-1");
    CHECK(!ctr_is_forward(max - 1, max), "ctr: MAX-1 < MAX (backward)");
}

static void test_ctr_half_range_boundary(void)
{
    uint64_t half = UINT64_C(1) << 63;
    CHECK(!ctr_is_forward(half, 0),      "ctr: exactly 2^63 away = not forward");
    CHECK( ctr_is_forward(half - 1, 0),  "ctr: 2^63-1 away = forward");
    CHECK(!ctr_is_forward(0, half),      "ctr: backward jump of 2^63");
}

static void test_kv_basic(void)
{
    CHECK( kv_is_forward(0, 0),   "kv: 0 == 0 (equal ok)");
    CHECK( kv_is_forward(1, 0),   "kv: 1 > 0");
    CHECK( kv_is_forward(1, 1),   "kv: 1 == 1 (equal ok)");
    CHECK( kv_is_forward(100, 1), "kv: 100 > 1");
    CHECK(!kv_is_forward(0, 1),   "kv: 0 < 1 (backward)");
    CHECK(!kv_is_forward(5, 10),  "kv: 5 < 10 (backward)");
}

static void test_kv_wrap(void)
{
    uint32_t max = UINT32_MAX;
    CHECK( kv_is_forward(0, max),   "kv: 0 wraps past MAX");
    CHECK( kv_is_forward(1, max),   "kv: 1 wraps past MAX");
    CHECK( kv_is_forward(max, max - 1), "kv: MAX > MAX-1");
    CHECK( kv_is_forward(max, max),     "kv: MAX == MAX (equal ok)");
    CHECK(!kv_is_forward(max - 1, max), "kv: MAX-1 < MAX (backward)");
}

static void test_kv_half_range_boundary(void)
{
    uint32_t half = UINT32_C(1) << 31;
    CHECK( kv_is_forward(half, 0),       "kv: exactly 2^31 away = forward (<=)");
    CHECK(!kv_is_forward(half + 1, 0),   "kv: 2^31+1 away = not forward");
    CHECK( kv_is_forward(half - 1, 0),   "kv: 2^31-1 away = forward");
}

int main(void)
{
    test_ctr_basic();
    test_ctr_gaps();
    test_ctr_wrap();
    test_ctr_half_range_boundary();
    test_kv_basic();
    test_kv_wrap();
    test_kv_half_range_boundary();

    printf("PEP forward-progress tests: %d passed, %d failed\n",
           pass_count, fail_count);
    return fail_count ? 1 : 0;
}
