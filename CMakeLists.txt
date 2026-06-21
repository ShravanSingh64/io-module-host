cmake_minimum_required(VERSION 3.20)
project(io_module_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_compile_options(-Wall -Wextra -Wpedantic)

# Header/source library (grows as we add protocol, parser, transport).
add_library(iomh STATIC
    src/frame_parser.cpp
    src/simulator.cpp
    src/host.cpp)
target_include_directories(iomh PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- Tests ---
include(FetchContent)
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.11
)
FetchContent_MakeAvailable(doctest)

enable_testing()
add_executable(test_crc unit_tests/test_crc.cpp)
target_link_libraries(test_crc PRIVATE iomh doctest::doctest)
add_test(NAME test_crc COMMAND test_crc)

add_executable(test_parser unit_tests/test_parser.cpp)
target_link_libraries(test_parser PRIVATE iomh doctest::doctest)
add_test(NAME test_parser COMMAND test_parser)

add_executable(host src/main.cpp)
target_link_libraries(host PRIVATE iomh)