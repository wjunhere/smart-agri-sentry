#include <stdio.h>
#include <string.h>
#include "lora_frame.h"

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_EQ(got, expected) do { \
    tests_run++; \
    if ((got) == (expected)) { tests_passed++; } \
    else { printf("FAIL: %s:%d got %d expected %d\n", __FILE__, __LINE__, (int)(got), (int)(expected)); } \
} while(0)

static void test_pack_data(void)
{
    cj702_data_t avg = {303, 20, 120, 35, 50, 2500, 6000};
    uint8_t buf[32];
    int len = lora_frame_pack_data(0x01, &avg, buf, sizeof(buf));
    ASSERT_EQ(len, 20);
    ASSERT_EQ(buf[0], 0xAA);
    ASSERT_EQ(buf[1], 0x55);
    ASSERT_EQ(buf[2], 0x01);
    ASSERT_EQ(buf[3], LORA_MSG_DATA);
    ASSERT_EQ(buf[4], 14);
}

static void test_pack_error(void)
{
    uint8_t buf[32];
    int len = lora_frame_pack_error(0x01, LORA_ERROR_TIMEOUT, buf, sizeof(buf));
    ASSERT_EQ(len, 7);
    ASSERT_EQ(buf[3], LORA_MSG_ERROR);
    ASSERT_EQ(buf[4], 1);
    ASSERT_EQ(buf[5], LORA_ERROR_TIMEOUT);
}

int test_lora_frame(void)
{
    test_pack_data();
    test_pack_error();
    printf("lora_frame: %d/%d passed\n", tests_passed, tests_run);
    return tests_run == tests_passed ? 0 : 1;
}
