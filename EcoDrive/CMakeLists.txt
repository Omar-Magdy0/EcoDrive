cmake_minimum_required(VERSION 3.20)


# Select target platform
set(PLATFORM_NAME "host" CACHE STRING "Target platform (host, stm32f4)")
set(ECOLIB_DIR ${CMAKE_SOURCE_DIR}/../ecolib)
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/../thirdparty)
set(PMSMCONTROL_DIR ${CMAKE_SOURCE_DIR}/PmsmControl)

####platform specific settings
# SET TOOLCHAIN BEFORE PROJECT - ADD THIS LINE
include(platform/${PLATFORM_NAME}/platform.cmake)
project(EcoDrive LANGUAGES C CXX ASM)

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "
    In-source builds are not allowed!
    Please use:
    mkdir build && cd build && cmake ..
    ")
endif()

# Setup cmake module path and compiler settings
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set C++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Control where files go WITHIN the build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${ECOLIB_DIR}
    ${ECOLIB_DIR}/middleware
    ${PMSMCONTROL_DIR}
)


# Link with platform EARLY to get include paths
add_subdirectory(platform/${PLATFORM_NAME})

message(STATUS "LIBRARIES TO LINK: ${PLATFORM_LIB}")

# Add platform includes to global include path
include_directories(${PLATFORM_INCLUDES})

file(GLOB_RECURSE MIDDLEWARE_SRC    
    ${ECOLIB_DIR}/middleware/*.cpp
    ${ECOLIB_DIR}/middleware/*.c
)



set(APPLICATION_SRC
    ${PMSMCONTROL_DIR}/PmsmControl.cpp
    ${PMSMCONTROL_DIR}/PosDriver.cpp
    ${PMSMCONTROL_DIR}/PmsmControlCore.cpp
    ${PMSMCONTROL_DIR}/PmsmControlCore_Foc.cpp
    ${PMSMCONTROL_DIR}/PmsmControlCore_SComm.cpp
    ${CMAKE_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/sys.cpp
)


file(GLOB_RECURSE CORE_SRC
    ${ECOLIB_DIR}/el/*.cpp
    ${ECOLIB_DIR}/el/*.c
)


# Create executable
add_executable(${PROJECT_NAME} ${APPLICATION_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})

# Add eld sources directly to the executable to ensure strong IRQ handlers
# override weak startup symbols (avoids linker archive symbol resolution issues)
target_sources(${PROJECT_NAME} PRIVATE ${PLATFORM_SOURCE})

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PLATFORM_INTERFACE}
    ${PLATFORM_LIB}
    ${TOOLCHAIN_LINK_LIBRARIES}
)

# BUILD Platform Tied TEST CODE
if(PLATFORM_NAME STREQUAL "host")
    add_definitions(-DPLATFORM_HOST)

# ── Unit Tests (optional) ────────────────────────────────────
option(TESTS_BUILD "Build unit test" OFF)
if(TESTS_BUILD)
    # Set third-party directory at root level
    set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)
    file(MAKE_DIRECTORY ${THIRDPARTY_DIR})
    
    add_subdirectory(test)
    message(STATUS "Unit test enabled. Use 'ctest' to run.")
    
    # Add convenience targets at root level
    add_custom_target(build_tests
        COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all_unit_tests
        COMMENT "Building all unit test..."
    )
    
    add_custom_target(run_tests
        COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target run_all_tests
        COMMENT "Running all unit test..."
    )
endif()

endif()



if(NOT PLATFORM_NAME STREQUAL "host")
# ADD THIS: Show size automatically after every build
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "=== Memory Usage ==="
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Displaying memory usage"
)

# Size target for checking memory usage
add_custom_target(size
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E echo "=== Detailed Memory Analysis ==="
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)
endif()
