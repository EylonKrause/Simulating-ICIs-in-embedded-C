# ===========================================================================
#  CMakeLists.txt -- portable build for Linux, macOS and Windows.
#
#      cmake -B build && cmake --build build && ./build/test_all
#
#  No find_package, no external dependencies: the project uses only the C
#  standard library.
# ===========================================================================
cmake_minimum_required(VERSION 3.16)
project(simulating_icis C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

file(GLOB LIB_SOURCES ${CMAKE_SOURCE_DIR}/src/*.c)
add_library(icis STATIC ${LIB_SOURCES})
target_include_directories(icis PUBLIC ${CMAKE_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(icis PRIVATE /W4)
    target_compile_definitions(icis PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(icis PRIVATE -Wall -Wextra -Wpedantic)
    find_library(M_LIB m)
    if(M_LIB)
        target_link_libraries(icis PUBLIC ${M_LIB})
    endif()
endif()

file(GLOB APP_SOURCES ${CMAKE_SOURCE_DIR}/apps/*.c)
foreach(app_src ${APP_SOURCES})
    get_filename_component(app_name ${app_src} NAME_WE)
    add_executable(${app_name} ${app_src})
    target_link_libraries(${app_name} PRIVATE icis)
    if(MSVC)
        target_compile_definitions(${app_name} PRIVATE _CRT_SECURE_NO_WARNINGS)
    endif()
endforeach()

enable_testing()
add_executable(test_all ${CMAKE_SOURCE_DIR}/tests/test_all.c)
target_link_libraries(test_all PRIVATE icis)
add_test(NAME unit_tests COMMAND test_all)
