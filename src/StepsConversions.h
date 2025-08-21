//
// Created by paulw on 8/4/2025.
//

#ifndef STEPS_CONVERSIONS_H
#define STEPS_CONVERSIONS_H
#include "RsTypeNames.h"

#define ABS(x) ((x) < 0 ? -(x) : (x))


static constexpr f64 HALF = 0.5;
static constexpr i64 round_to_nearest(const f64 x) {
    return x < 0 ? static_cast<i64>(x - HALF) : static_cast<i64>(x + HALF);
}


static_assert(round_to_nearest(0.2) == 0, "round error");
static_assert(round_to_nearest(0.5) == 1, "round error");
static_assert(round_to_nearest(0.51) == 1, "round error");
static_assert(round_to_nearest(51) == 51, "round error");
static_assert(round_to_nearest(-0.2) == -0, "round error");
static_assert(round_to_nearest(-0.5) == -1, "round error");
static_assert(round_to_nearest(-0.51) == -1, "round error");
static_assert(round_to_nearest(-51) == -51, "round error");

static constexpr i64 FIXED_BITS = 32;
static constexpr i64 FIXED_DENOMINATOR = 1ll << FIXED_BITS;
// hundreths to steps
static constexpr f64 HTS_RATIO = 13.4751185151138; // CTU: 13.4751185151138 | Slide: 40.97
// hundreths to steps fixed fraction
static constexpr i64 FIXED_HTS_RATIO = round_to_nearest(HTS_RATIO * FIXED_DENOMINATOR);
// steps to hundreths
static constexpr f64 STH_RATIO = 1.0/HTS_RATIO;
// steps to hundreths fixed fraction
static constexpr i64 FIXED_STH_RATIO = round_to_nearest(STH_RATIO * FIXED_DENOMINATOR);

/**
 * convert steps at motor to hundreths of an inch of travel
 * @param steps input range [-2^22..2^22]
 */
static constexpr i32 steps_to_hundreths(const i32 steps) {
    return static_cast<i32>(((static_cast<i64>(steps) * FIXED_STH_RATIO)+(1ll<<(FIXED_BITS-1))) >> FIXED_BITS);
}

/**
 * convert hundreths of an inch of travel to steps at motor
 * @param hundreths input range [-2^22..2^22]
 */
static constexpr i32 hundreths_to_steps(const i32 hundreths) {
    return static_cast<i32>(((static_cast<i64>(hundreths) * FIXED_HTS_RATIO)+(1ll<<(FIXED_BITS-1))) >> FIXED_BITS);
}

/**
 * Self explanatory, only used to verify fast integer math alternative
 */
static constexpr i32 slow_steps_to_hundreths(const i32 steps) {
    return static_cast<i32>(round_to_nearest(static_cast<f64>(steps) / HTS_RATIO));
}
/**
 * Self explanatory, only used to verify fast integer math alternative
 */
static constexpr i32 slow_hundreths_to_steps(const i32 hundreths) {
    return static_cast<i32>(round_to_nearest(static_cast<f64>(hundreths) * HTS_RATIO));
}

// hundreths per minute to steps per second
static constexpr f64 HPM_TO_SPS_RATIO = HTS_RATIO/60;
// hundreths per minute to steps per second fixed fraction
static constexpr i64 FIXED_HPM_TO_SPS_RATIO = static_cast<i64>(round_to_nearest(HPM_TO_SPS_RATIO * FIXED_DENOMINATOR));
// steps per second to hundreths per minute
static constexpr f64 SPS_TO_HPM_RATIO = 1/HPM_TO_SPS_RATIO;
// steps per second to hundreths per minute fixed fraction
static constexpr i64 FIXED_SPS_TO_HPM_RATIO = static_cast<i64>(round_to_nearest(SPS_TO_HPM_RATIO * FIXED_DENOMINATOR));


/**
 * Steps per second to hundreths per minute
 * @param steps_per_second input range [-2^22..2^22]
 */
static constexpr i32 sps_to_hpm(const i32 steps_per_second) {
    return static_cast<i32>(((static_cast<i64>(steps_per_second) * FIXED_SPS_TO_HPM_RATIO)+(1ll<<(FIXED_BITS-1))) >> FIXED_BITS);
}
/**
 * Hundreths per minute to steps per second
 * @param hundreths_per_minute input range [-2^22..2^22]
 */
static constexpr i32 hpm_to_sps(const i32 hundreths_per_minute) {
    return static_cast<i32>(((static_cast<i64>(hundreths_per_minute) * FIXED_HPM_TO_SPS_RATIO)+(1ll<<(FIXED_BITS-1))) >> FIXED_BITS);
}

/**
 * Steps per second to hundreths per minute
 *
 * Slow version, only used to verify fast integer math alternative
 */
static constexpr i32 slow_sps_to_hpm(const i32 steps_per_second) {
    return static_cast<i32>(round_to_nearest(static_cast<f64>(steps_per_second) * SPS_TO_HPM_RATIO));
}
/**
 * Hundreths per minute to steps per second
 *
 * Slow version, only used to verify fast integer math alternative
 */
static constexpr i32 slow_hpm_to_sps(const i32 hundreths_per_minute) {
    return static_cast<i32>(round_to_nearest(static_cast<f64>(hundreths_per_minute) * HPM_TO_SPS_RATIO));
}

#define TEST_DIST_CONVERSIONS_WITH_VAL(VAL) \
static_assert(ABS(steps_to_hundreths(hundreths_to_steps(VAL)) - VAL) <= round_to_nearest((1/HTS_RATIO)/2), "Step conversion failed"); \
static_assert(ABS(hundreths_to_steps(steps_to_hundreths(VAL)) - VAL) <= round_to_nearest(HTS_RATIO/2), "Step conversion failed"); \
static_assert(ABS(steps_to_hundreths(VAL) - slow_steps_to_hundreths(VAL)) <= 0, "Step conversion failed"); \
static_assert(ABS(hundreths_to_steps(VAL) - slow_hundreths_to_steps(VAL)) <= 0, "Step conversion failed");

#define TEST_SPEED_CONVERSIONS_WITH_VAL(VAL) \
static_assert(ABS(sps_to_hpm(hpm_to_sps(VAL)) - VAL) <= round_to_nearest((1/HPM_TO_SPS_RATIO)/2), "Step conversion failed 1");  \
static_assert(ABS(hpm_to_sps(sps_to_hpm(VAL)) - VAL) <= round_to_nearest(HPM_TO_SPS_RATIO/2), "Step conversion failed 2");  \
static_assert(ABS(sps_to_hpm(VAL) - slow_sps_to_hpm(VAL)) <= 0, "Step conversion failed 3");  \
static_assert(ABS(hpm_to_sps(VAL) - slow_hpm_to_sps(VAL)) <= 0, "Step conversion failed 4");

TEST_SPEED_CONVERSIONS_WITH_VAL(489000)
TEST_SPEED_CONVERSIONS_WITH_VAL(489)
TEST_SPEED_CONVERSIONS_WITH_VAL(1)
TEST_SPEED_CONVERSIONS_WITH_VAL(0)
TEST_SPEED_CONVERSIONS_WITH_VAL(80000000)
TEST_SPEED_CONVERSIONS_WITH_VAL(-489000)
TEST_SPEED_CONVERSIONS_WITH_VAL(-489)
TEST_SPEED_CONVERSIONS_WITH_VAL(-1)
TEST_SPEED_CONVERSIONS_WITH_VAL(-0)
TEST_SPEED_CONVERSIONS_WITH_VAL(-80000000)

TEST_DIST_CONVERSIONS_WITH_VAL(489000)
TEST_DIST_CONVERSIONS_WITH_VAL(489)
TEST_DIST_CONVERSIONS_WITH_VAL(1)
TEST_DIST_CONVERSIONS_WITH_VAL(0)
TEST_DIST_CONVERSIONS_WITH_VAL(800000)
TEST_DIST_CONVERSIONS_WITH_VAL(-489000)
TEST_DIST_CONVERSIONS_WITH_VAL(-489)
TEST_DIST_CONVERSIONS_WITH_VAL(-1)
TEST_DIST_CONVERSIONS_WITH_VAL(-0)
TEST_DIST_CONVERSIONS_WITH_VAL(-800000)


TEST_DIST_CONVERSIONS_WITH_VAL((1<<22))
TEST_DIST_CONVERSIONS_WITH_VAL(-(1<<22))

// Prime numbers
TEST_DIST_CONVERSIONS_WITH_VAL(2)
TEST_DIST_CONVERSIONS_WITH_VAL(3)
TEST_DIST_CONVERSIONS_WITH_VAL(5)
TEST_DIST_CONVERSIONS_WITH_VAL(7)
TEST_DIST_CONVERSIONS_WITH_VAL(11)
TEST_DIST_CONVERSIONS_WITH_VAL(97)

// Powers of 2 (important for binary operations)
TEST_DIST_CONVERSIONS_WITH_VAL(2)
TEST_DIST_CONVERSIONS_WITH_VAL(4)
TEST_DIST_CONVERSIONS_WITH_VAL(8)
TEST_DIST_CONVERSIONS_WITH_VAL(16)
TEST_DIST_CONVERSIONS_WITH_VAL(32)
TEST_DIST_CONVERSIONS_WITH_VAL(64)
TEST_DIST_CONVERSIONS_WITH_VAL(128)
TEST_DIST_CONVERSIONS_WITH_VAL(256)
TEST_DIST_CONVERSIONS_WITH_VAL(512)
TEST_DIST_CONVERSIONS_WITH_VAL(1024)
TEST_DIST_CONVERSIONS_WITH_VAL(2048)
TEST_DIST_CONVERSIONS_WITH_VAL(4096)
TEST_DIST_CONVERSIONS_WITH_VAL(8192)

// Values around HTS_RATIO
TEST_DIST_CONVERSIONS_WITH_VAL(67)
TEST_DIST_CONVERSIONS_WITH_VAL(68)
TEST_DIST_CONVERSIONS_WITH_VAL(134)
TEST_DIST_CONVERSIONS_WITH_VAL(135)

// Common values in industrial settings
TEST_DIST_CONVERSIONS_WITH_VAL(1000)
TEST_DIST_CONVERSIONS_WITH_VAL(10000)
TEST_DIST_CONVERSIONS_WITH_VAL(100000)
TEST_DIST_CONVERSIONS_WITH_VAL(1000000)

// Similar tests for speed conversions
TEST_SPEED_CONVERSIONS_WITH_VAL((1<<24))
TEST_SPEED_CONVERSIONS_WITH_VAL(-(1<<24))

// Prime numbers for speed
TEST_SPEED_CONVERSIONS_WITH_VAL(2)
TEST_SPEED_CONVERSIONS_WITH_VAL(3)
TEST_SPEED_CONVERSIONS_WITH_VAL(5)
TEST_SPEED_CONVERSIONS_WITH_VAL(7)
TEST_SPEED_CONVERSIONS_WITH_VAL(11)
TEST_SPEED_CONVERSIONS_WITH_VAL(97)

// Powers of 2 for speed
TEST_SPEED_CONVERSIONS_WITH_VAL(2)
TEST_SPEED_CONVERSIONS_WITH_VAL(4)
TEST_SPEED_CONVERSIONS_WITH_VAL(16)
TEST_SPEED_CONVERSIONS_WITH_VAL(64)
TEST_SPEED_CONVERSIONS_WITH_VAL(256)
TEST_SPEED_CONVERSIONS_WITH_VAL(1024)
TEST_SPEED_CONVERSIONS_WITH_VAL(4096)

// Values around HPM_TO_SPS_RATIO
TEST_SPEED_CONVERSIONS_WITH_VAL(1)
TEST_SPEED_CONVERSIONS_WITH_VAL(60)
TEST_SPEED_CONVERSIONS_WITH_VAL(120)
TEST_SPEED_CONVERSIONS_WITH_VAL(600)

// Typical industrial speeds
TEST_SPEED_CONVERSIONS_WITH_VAL(3000)
TEST_SPEED_CONVERSIONS_WITH_VAL(6000)
TEST_SPEED_CONVERSIONS_WITH_VAL(12000)
TEST_SPEED_CONVERSIONS_WITH_VAL(60000)
TEST_SPEED_CONVERSIONS_WITH_VAL(300000)


#endif //STEPS_CONVERSIONS_H
