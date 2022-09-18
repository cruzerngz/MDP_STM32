cmake_minimum_required(VERSION 3.10)

project(test_state_machine)

include_directories(
    ../lib/greatest
    .
)

add_executable(
    ${PROJECT_NAME}
    main.c
    uart_state_machine.c
)
