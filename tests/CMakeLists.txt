cmake_minimum_required(VERSION 3.10)

project(
    test_stm32
    C
    DESCRIPTION Unit tests for code used in the STM32
)

include_directories( ## this doesnt propogate the include path down to other dirs????
    "lib/greatest"
)

add_subdirectory(lib)
add_subdirectory(test_state_machine)
