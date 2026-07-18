# Unit Tests CMake Configuration
# ==============================
# Purpose: Build and run unit tests with Catch2
# Usage: cmake -B build -DTESTS_BUILD=ON && cmake --build build

cmake_minimum_required(VERSION 3.16)
set(PLATFORM_NAME "host" CACHE STRING "Target platform (host, stm32f4)")
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/../thirdparty)
set(ECOLIB_DIR ${CMAKE_SOURCE_DIR}/../ecolib)
# Only include if this is included as a subdirectory
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Standalone build mode
    project(EcoDriveTests LANGUAGES CXX VERSION 1.0.0)
endif()

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ── Dependencies ─────────────────────────────────────────────
# Option 1: FetchContent (recommended for reproducibility)
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY  https://github.com/catchorg/Catch2.git
    GIT_TAG         v3.8.1
)
FetchContent_MakeAvailable(Catch2)

# Option 2: Use find_package if installed system-wide
# find_package(Catch2 3 REQUIRED)
file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_SOURCE_DIR}/middleware/*.cpp"
)

file(GLOB_RECURSE CORE_SOURCES
    "${ECOLIB_DIR}/el/*.cpp"
)

file(GLOB_RECURSE MIDDLEWARE_SOURCES
    "${ECOLIB_DIR}/middleware/*.cpp"
)

add_executable(unit_tests
    ${CMAKE_SOURCE_DIR}/dummy_test.cpp
    ${TEST_SOURCES}
    ${CORE_SOURCES}
    ${MIDDLEWARE_SOURCES}
)

target_include_directories(unit_tests PRIVATE
    ${ECOLIB_DIR}
    ${CMAKE_SOURCE_DIR}
    ${ECOLIB_DIR}/middleware
    ${THIRDPARTY_DIR}
)

target_link_libraries(unit_tests PRIVATE
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(unit_tests)
