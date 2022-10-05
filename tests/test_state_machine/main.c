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
    ASSERT_EQ(FLAG_MOVE_DIR, 1);

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
    ASSERT_EQ(FLAG_MOVE_DIR, -1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_front_left() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('l'), '.'); // set turn direction
    ASSERT_EQ(state_machine_interpreter('f'), '_'); // turn, set mvmt dir
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_ANGLE, 100);
    ASSERT_EQ(FLAG_TURN_DIR, -1);
    ASSERT_EQ(FLAG_MOVE_DIR, 1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_front_right() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('r'), '.'); // set turn direction
    ASSERT_EQ(state_machine_interpreter('f'), '_'); // turn, set mvmt dir
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_ANGLE, 100);
    ASSERT_EQ(FLAG_TURN_DIR, 1);
    ASSERT_EQ(FLAG_MOVE_DIR, 1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_back_left() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('l'), '.'); // set turn direction
    ASSERT_EQ(state_machine_interpreter('b'), '_'); // turn, set mvmt dir
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_ANGLE, 100);
    ASSERT_EQ(FLAG_TURN_DIR, -1);
    ASSERT_EQ(FLAG_MOVE_DIR, -1);

    PASS();
}

/**
 * @brief Test turning for fine control
 *
 * @return TEST
 */
TEST test_fine_control_turn_back_right() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('t'), '.'); // turn, await subcommand
    ASSERT_EQ(state_machine_interpreter('r'), '.'); // set turn direction
    ASSERT_EQ(state_machine_interpreter('b'), '_'); // turn, set mvmt dir
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter('0'), '_'); // move, continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // move, complete entry

    // check for correct value in global
    ASSERT_EQ(FLAG_TURN_ANGLE, 100);
    ASSERT_EQ(FLAG_TURN_DIR, 1);
    ASSERT_EQ(FLAG_MOVE_DIR, -1);

    PASS();
}

/**
 * @brief Tests the in-place cardinal turns
 *
 * @return TEST
 */
TEST test_fine_control_in_place_cardinal_left() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('c'), '_'); // in-place turn cardinal
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('1'), '_'); // continue val entry
    ASSERT_EQ(state_machine_interpreter('3'), '_'); // continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // complete, exec

    // check for correct values in global
    ASSERT_EQ(FLAG_IN_PLACE_CARDINAL, 13);

    PASS();
}

/**
 * @brief Tests the in-place cardinal turns
 *
 * @return TEST
 */
TEST test_fine_control_in_place_cardinal_right() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('c'), '_'); // in-place turn cardinal
    ASSERT_EQ(GLOBAL_FINE_CONTROL_MAGNITUDE, 0); // check that value is zeroed

    ASSERT_EQ(state_machine_interpreter('4'), '_'); // continue val entry
    ASSERT_EQ(state_machine_interpreter(';'), ';'); // complete, exec

    // check for correct values in global
    ASSERT_EQ(FLAG_IN_PLACE_CARDINAL, 4);

    PASS();
}

TEST test_fine_control_approach_obstacle() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';'); // return to mode select
    ASSERT_EQ(state_machine_interpreter('f'), ';'); // enter fine control
    ASSERT_EQ(state_machine_interpreter('o'), ';'); // appr obstacle

    ASSERT_EQ(FLAG_APPROACH_OBSTACLE, 1);

    PASS();
}

/**
 * @brief Test the modification of KP
 *
 * @return TEST
 */
TEST test_config_set_kp() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('k'), '.');
    ASSERT_EQ(state_machine_interpreter('p'), '_');
    ASSERT_EQ(state_machine_interpreter('1'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter(';'), ';');

    PASS();
}

/**
 * @brief Test the modification of KI
 *
 * @return TEST
 */
TEST test_config_set_ki() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('k'), '.');
    ASSERT_EQ(state_machine_interpreter('i'), '_');
    ASSERT_EQ(state_machine_interpreter('1'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter(';'), ';');

    PASS();
}

/**
 * @brief Test the modification of KD
 *
 * @return TEST
 */
TEST test_config_set_kd() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('k'), '.');
    ASSERT_EQ(state_machine_interpreter('d'), '_');
    ASSERT_EQ(state_machine_interpreter('1'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter(';'), ';');
    
    PASS();
}

/**
 * @brief Test the modification of default straight speed
 *
 * @return TEST
 */
TEST test_config_set_straight_speed() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('s'), '.');
    ASSERT_EQ(state_machine_interpreter('s'), '_');
    ASSERT_EQ(state_machine_interpreter('2'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter(';'), ';');
    
    PASS();
}

/**
 * @brief Test the modification of default turn speed
 *
 * @return TEST
 */
TEST test_config_set_turn_speed() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('s'), '.');
    ASSERT_EQ(state_machine_interpreter('t'), '_');
    ASSERT_EQ(state_machine_interpreter('1'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter('0'), '_');
    ASSERT_EQ(state_machine_interpreter(';'), ';');
    
    PASS();
}

/**
 * @brief Test the setting of low grip surface flag
 *
 * @return TEST
 */
TEST test_config_set_low_grip_flag() {
    ASSERT_EQ(state_machine_interpreter('\\'), ';');
    ASSERT_EQ(state_machine_interpreter('c'), ';');
    ASSERT_EQ(state_machine_interpreter('g'), '.');
    ASSERT_EQ(state_machine_interpreter('t'), ';'); // t for true, f for false
    
    PASS();
}

/**
 * @brief Construct a new SUITE object
 * Add tests to be run here
 *
 */
SUITE(suite_fine_control) {
    // forward, backward
    RUN_TEST(test_fine_control_move_forward);
    RUN_TEST(test_fine_control_move_backward);

    // turns, all kinds
    RUN_TEST(test_fine_control_turn_front_left);
    RUN_TEST(test_fine_control_turn_front_right);
    RUN_TEST(test_fine_control_turn_back_left);
    RUN_TEST(test_fine_control_turn_back_right);

    // cardinal in-place turns
    RUN_TEST(test_fine_control_in_place_cardinal_left);
    RUN_TEST(test_fine_control_in_place_cardinal_right);

    RUN_TEST(test_fine_control_approach_obstacle);
}

/**
 * @brief Construct a new SUITE object
 * Tests for the config sub-state machine
 */
SUITE(suite_config) {
    RUN_TEST(test_config_set_kd);
    RUN_TEST(test_config_set_ki);
    RUN_TEST(test_config_set_kp);
    RUN_TEST(test_config_set_straight_speed);
    RUN_TEST(test_config_set_turn_speed);
    RUN_TEST(test_config_set_low_grip_flag);
}

GREATEST_MAIN_DEFS(); // important, right before main

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();

    RUN_SUITE(suite_fine_control);
    RUN_SUITE(suite_config);

    GREATEST_MAIN_END();
    return 0;
}
