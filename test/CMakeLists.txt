# Unit Tests CMake Configuration
# ==============================
# Purpose: Build and run unit tests with Catch2
# Usage: cmake -B build -DTESTS_BUILD=ON && cmake --build build

cmake_minimum_required(VERSION 3.16)

# Only include if this is included as a subdirectory
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Standalone build mode
    project(EcoDriveTests LANGUAGES CXX VERSION 1.0.0)
endif()

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

# ── Library targets for modules under test ──────────────────
# Core module (platform-independent utilities)
add_library(test_middleware STATIC
    ${CMAKE_SOURCE_DIR}/middleware/Codec.h
    ${CMAKE_SOURCE_DIR}/middleware/IDV.h
    ${CMAKE_SOURCE_DIR}/middleware/DAQStream.h
    ${CMAKE_SOURCE_DIR}/middleware/DAQStream.cpp
)
target_include_directories(test_middleware PUBLIC 
    ${CMAKE_SOURCE_DIR}/middleware
)

target_compile_features(test_middleware PUBLIC cxx_std_20)

# ── Test executables ────────────────────────────────────────
# Helper macro to create test targets
macro(add_test_target TARGET_NAME SOURCE_FILES LINK_LIBS)
    add_executable(${TARGET_NAME} ${SOURCE_FILES})
    target_link_libraries(${TARGET_NAME} PRIVATE ${LINK_LIBS} Catch2::Catch2WithMain)
    target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
    
    # Add CTest integration
    include(CTest)
    include(Catch)
    catch_discover_tests(${TARGET_NAME})
endmacro()

# Core tests
file(GLOB_RECURSE CORE_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp
)
if(CORE_TEST_SOURCES)
    add_test_target(test_middleware_suite "${CORE_TEST_SOURCES}" test_middleware)
endif()

# Middleware tests
file(GLOB_RECURSE MIDDLEWARE_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/middleware/*.cpp
)
if(MIDDLEWARE_TEST_SOURCES)
    add_test_target(test_middleware_suite "${MIDDLEWARE_TEST_SOURCES}" test_middleware)
endif()

# Dummy test (always build)
add_test_target(test_dummy "${CMAKE_CURRENT_SOURCE_DIR}/dummy_test.cpp" test_middleware)

# ── Coverage options (optional) ─────────────────────────────
option(TESTS_COVERAGE "Enable code coverage reporting" OFF)
if(TESTS_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
        add_link_options(--coverage)
    endif()
endif()

message(STATUS "Test targets:")
message(STATUS "  test_dummy           - Basic sanity check")
foreach(test_target IN ITEMS ${CORE_TEST_SOURCES} ${MIDDLEWARE_TEST_SOURCES})
    get_filename_component(test_name ${test_target} NAME_WE)
    message(STATUS "  test_${test_name}    - ${test_target}")
endforeach()
