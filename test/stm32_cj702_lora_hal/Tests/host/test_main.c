#include <stdio.h>

extern int test_cj702(void);
extern int test_aggregator(void);
extern int test_lora_frame(void);

int main(void)
{
    int rc = 0;
    rc |= test_cj702();
    rc |= test_aggregator();
    rc |= test_lora_frame();
    return rc;
}
