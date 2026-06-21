#include <stdio.h>
#include <string.h>
#include "cj702.h"

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_EQ(got, expected) do { \
    tests_run++; \
    if ((got) == (expected)) { tests_passed++; } \
    else { printf("FAIL: %s:%d got %d expected %d\n", __FILE__, __LINE__, (int)(got), (int)(expected)); } \
} while(0)

static void test_parse_valid_positive_temp(void)
{
    // co2=0x012F=303, hcho=0x0014=20, tvoc=0x0078=120, pm25=0x0023=35, pm10=0x0032=50
    // temp=+27.00 => B13=0x1B, B14=0x00; humidity=60.00 => B15=0x3C, B16=0x00
    uint8_t frame[CJ702_FRAME_LEN] = {
        0x3C, 0x02,
        0x01, 0x2F,
        0x00, 0x14,
        0x00, 0x78,
        0x00, 0x23,
        0x00, 0x32,
        0x1B, 0x00,
        0x3C, 0x00,
        0x00
    };
    uint16_t sum = 0;
    for (int i = 0; i < CJ702_FRAME_LEN - 1; i++) sum += frame[i];
    frame[CJ702_FRAME_LEN - 1] = (uint8_t)(sum & 0xFF);

    cj702_data_t d;
    ASSERT_EQ(cj702_parse(frame, &d), true);
    ASSERT_EQ(d.co2, 303);
    ASSERT_EQ(d.hcho, 20);
    ASSERT_EQ(d.tvoc, 120);
    ASSERT_EQ(d.pm25, 35);
    ASSERT_EQ(d.pm10, 50);
    ASSERT_EQ(d.temp, 2700);
    ASSERT_EQ(d.humidity, 6000);
}

static void test_parse_valid_negative_temp(void)
{
    uint8_t frame[CJ702_FRAME_LEN] = {
        0x3C, 0x02,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x9B, 0x00,  // -27
        0x00, 0x00,
        0x00
    };
    uint16_t sum = 0;
    for (int i = 0; i < CJ702_FRAME_LEN - 1; i++) sum += frame[i];
    frame[CJ702_FRAME_LEN - 1] = (uint8_t)(sum & 0xFF);

    cj702_data_t d;
    ASSERT_EQ(cj702_parse(frame, &d), true);
    ASSERT_EQ(d.temp, -2700);
}

static void test_parse_bad_checksum(void)
{
    uint8_t frame[CJ702_FRAME_LEN] = {0x3C, 0x02};
    memset(frame + 2, 0, CJ702_FRAME_LEN - 3);
    frame[CJ702_FRAME_LEN - 1] = 0xFF;
    cj702_data_t d;
    ASSERT_EQ(cj702_parse(frame, &d), false);
}

int test_cj702(void)
{
    test_parse_valid_positive_temp();
    test_parse_valid_negative_temp();
    test_parse_bad_checksum();
    printf("cj702: %d/%d passed\n", tests_passed, tests_run);
    return tests_run == tests_passed ? 0 : 1;
}
