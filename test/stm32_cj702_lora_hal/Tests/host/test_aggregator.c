#include <stdio.h>
#include "aggregator.h"

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_EQ(got, expected) do { \
    tests_run++; \
    if ((got) == (expected)) { tests_passed++; } \
    else { printf("FAIL: %s:%d got %d expected %d\n", __FILE__, __LINE__, (int)(got), (int)(expected)); } \
} while(0)

static void test_average(void)
{
    aggregator_t agg;
    aggregator_init(&agg);
    cj702_data_t s = {100, 10, 50, 20, 30, 2500, 5000};
    for (int i = 0; i < 30; i++) {
        aggregator_add(&agg, &s);
    }
    cj702_data_t out;
    ASSERT_EQ(aggregator_get_average(&agg, &out), true);
    ASSERT_EQ(out.co2, 100);
    ASSERT_EQ(out.temp, 2500);
    ASSERT_EQ(out.humidity, 5000);
}

static void test_too_few_samples(void)
{
    aggregator_t agg;
    aggregator_init(&agg);
    cj702_data_t s = {1, 2, 3, 4, 5, 6, 7};
    for (int i = 0; i < 5; i++) {
        aggregator_add(&agg, &s);
    }
    cj702_data_t out;
    ASSERT_EQ(aggregator_get_average(&agg, &out), false);
}

int test_aggregator(void)
{
    test_average();
    test_too_few_samples();
    printf("aggregator: %d/%d passed\n", tests_passed, tests_run);
    return tests_run == tests_passed ? 0 : 1;
}
