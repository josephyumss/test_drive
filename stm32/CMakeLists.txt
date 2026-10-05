cmake_minimum_required(VERSION 3.16)
project(guide_amr_stm32_core C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(amr_stm32_core
    Core/Src/amr_app.c
    Core/Src/amr_crc16.c
    Core/Src/amr_motor.c
    Core/Src/amr_protocol.c
    Core/Src/amr_safety.c
    Core/Src/amr_watchdog.c
    Core/Src/range_sensors.c
)

target_include_directories(amr_stm32_core PUBLIC
    Core/Inc
    ../protocol
)

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(amr_stm32_core PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()

enable_testing()
add_executable(amr_stm32_tests tests/test_amr.c)
target_link_libraries(amr_stm32_tests PRIVATE amr_stm32_core)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(amr_stm32_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
add_test(NAME amr_stm32_tests COMMAND amr_stm32_tests)
