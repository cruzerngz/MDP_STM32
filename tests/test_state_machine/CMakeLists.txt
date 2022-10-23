cmake_minimum_required(VERSION 3.10)

project(test_state_machine)

include_directories(
    ../lib/greatest
    ../../stm32/Core/Inc
    .
)

set(STM_CORE_SRC ../../stm32/Core/Src)

add_executable(
    ${PROJECT_NAME}
    main.c
    ${STM_CORE_SRC}/uart_state_machine.c
    ${STM_CORE_SRC}/globals.c
)

add_compile_definitions(UNITTEST) ## add macro definition to disable parts of uart_state_machine.c
