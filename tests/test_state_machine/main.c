#include <stdio.h>

#include "greatest.h"

#include "uart_state_machine.h"
#include "uart_state_machine_api.h"

extern uint16_t GLOBAL_FINE_CONTROL_MAGNITUDE;

/**
 * @brief Test the toy car mode
 *
 * @return TEST
 */
TEST test_fine_control() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('m'), '_'); // move, start enter value
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    printf("global fine control mag: %d\n", GLOBAL_FINE_CONTROL_MAGNITUDE);
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 1); // check that value is 1

    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is 10

    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry

    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_MOVEMENT_DISTANCE, 100);

    PASS();
}

SUITE(main_tests) {
    RUN_TEST(test_fine_control);
}

GREATEST_MAIN_DEFS(); // important, right before main

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();

    RUN_SUITE(main_tests);


    GREATEST_MAIN_END();
    return 0;
}
