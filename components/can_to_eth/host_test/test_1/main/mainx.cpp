#include <stdint.h>
#include <cstdio>

#include "unity.h"
#include "unity_test_runner.h"


extern "C" {

void app_main(void)
{
    UNITY_BEGIN();
    UNITY_END();
}

int main(void) {
    printf("HELLO MAYANK\n");
    app_main();
    return 0;
}

}