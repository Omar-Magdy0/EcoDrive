cmake_minimum_required(VERSION 3.20)


# Select target platform
set(PLATFORM_NAME "host" CACHE STRING "Target platform (host, stm32f4)")
####platform specific settings
# SET TOOLCHAIN BEFORE PROJECT - ADD THIS LINE
if(NOT PLATFORM_NAME STREQUAL "host")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/platform/${PLATFORM_NAME}/cmake/gcc-arm-none-eabi.cmake")
endif()

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
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/driver
    ${CMAKE_SOURCE_DIR}/middleware
    ${CMAKE_SOURCE_DIR}/core
)


# Link with platform EARLY to get include paths
if(PLATFORM_NAME STREQUAL "host")
   
    add_subdirectory(platform/host)
else()
   
    add_subdirectory(platform/stm32f4)
endif()
#add_library(tinycbor STATIC IMPORTED)
#set_target_properties(tinycbor PROPERTIES
#    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/middleware/tinycbor/build/libtinycbor.a
#)

message(STATUS "LIBRARIES TO LINK: ${PLATFORM_LIB}")

# Add platform includes to global include path
include_directories(${PLATFORM_INCLUDES})



set(MIDDLEWARE_SRC
    ${CMAKE_SOURCE_DIR}/middleware/aebfStream.c
)

file(GLOB_RECURSE APPLICATION_SRC
    "application/*.cpp"
    "application/*.c"
)

file(GLOB_RECURSE DRIVER_SRC
    "driver/*.cpp"
    "driver/*.c"
)
# Source files

file(GLOB_RECURSE CORE_SRC
    "core/*.cpp"
    "core/*.c"
)


# Create executable
add_executable(${PROJECT_NAME} ${APPLICATION_SRC} ${DRIVER_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})

# Add eld sources directly to the executable to ensure strong IRQ handlers
# override weak startup symbols (avoids linker archive symbol resolution issues)
target_sources(${PROJECT_NAME} PRIVATE ${PLATFORM_SOURCE})
#target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/middleware/tinycbor/src)
#target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/middleware/tinycbor/build)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PLATFORM_INTERFACE}
    ${PLATFORM_LIB}
    ${TOOLCHAIN_LINK_LIBRARIES}
    #tinycbor
)

# BUILD TEST CODE
if(PLATFORM_NAME STREQUAL "host")
    add_definitions(-DPLATFORM_HOST)
    add_executable(test_scopeStream ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scopeStream.cpp ${DRIVER_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})
    target_sources(test_scopeStream PRIVATE ${PLATFORM_SOURCE})
    target_link_libraries(test_scopeStream PRIVATE
        ${PLATFORM_INTERFACE}
        ${PLATFORM_LIB}
        ${TOOLCHAIN_LINK_LIBRARIES}
        #tinycbor
    )

    add_executable(test_inverter ${CMAKE_CURRENT_SOURCE_DIR}/test/test_inverter.cpp ${DRIVER_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})
    target_sources(test_inverter PRIVATE ${PLATFORM_SOURCE})
    target_link_libraries(test_inverter PRIVATE
        ${PLATFORM_INTERFACE}
        ${PLATFORM_LIB}
        ${TOOLCHAIN_LINK_LIBRARIES}
        #tinycbor
    )

    add_executable(test_pmsm ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pmsm.cpp ${DRIVER_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})
    target_sources(test_pmsm PRIVATE ${PLATFORM_SOURCE})
    target_link_libraries(test_pmsm PRIVATE
        ${PLATFORM_INTERFACE}
        ${PLATFORM_LIB}
        ${TOOLCHAIN_LINK_LIBRARIES}
        #tinycbor
    )

    add_executable(EcoTool ${CMAKE_CURRENT_SOURCE_DIR}/EcoTool/EcoTool.cpp ${DRIVER_SRC} ${CORE_SRC} ${MIDDLEWARE_SRC})
    target_sources(EcoTool PRIVATE ${PLATFORM_SOURCE})
    target_link_libraries(EcoTool PRIVATE
        ${PLATFORM_INTERFACE}
        ${PLATFORM_LIB}
        ${TOOLCHAIN_LINK_LIBRARIES}
        #tinycbor
    )
    set_target_properties(EcoTool PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/EcoTool
    )
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
