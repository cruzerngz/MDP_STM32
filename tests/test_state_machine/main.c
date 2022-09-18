#include <stdio.h>

#include "greatest.h"

#include "uart_state_machine.h"
#include "uart_state_machine_api.h"

extern uint16_t GLOBAL_FINE_CONTROL_MAGNITUDE;

/**
 * @brief Test movement for fine control
 *
 * @return TEST
 */
TEST test_fine_control_move_forward() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('m'), '.'); // move, await subcommand
    ASSERT_EQ(state_machine_interpreter('f'), '_'); // set move direction
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_MOVEMENT_DISTANCE, 100);
    ASSERT_EQ(FLAG_DIRECTION, 1);

    PASS();
}

/**
 * @brief Test movement for fine control
 *
 * @return TEST
 */
TEST test_fine_control_move_backward() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('m'), '.'); // move, await subcommand
    ASSERT_EQ(state_machine_interpreter('b'), '_'); // set move direction
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_MOVEMENT_DISTANCE, 100);
    ASSERT_EQ(FLAG_DIRECTION, -1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_left() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('l'), '_'); // set turn direction
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_DIRECTION, 100);
    ASSERT_EQ(FLAG_DIRECTION, -1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_right() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('r'), '_'); // set turn direction
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_DIRECTION, 100);
    ASSERT_EQ(FLAG_DIRECTION, 1);

    PASS();
}

/**
 * @brief Construct a new SUITE object
 * Add tests to be run here
 *
 */
SUITE(suite_fine_control) {
    RUN_TEST(test_fine_control_move_forward);
    RUN_TEST(test_fine_control_move_backward);
    RUN_TEST(test_fine_control_turn_left);
    RUN_TEST(test_fine_control_turn_right);
}

GREATEST_MAIN_DEFS(); // important, right before main

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();

    RUN_SUITE(suite_fine_control);


    GREATEST_MAIN_END();
    return 0;
}
