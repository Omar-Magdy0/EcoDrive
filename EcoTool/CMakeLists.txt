cmake_minimum_required(VERSION 3.10)

# Project name and version
project(EcoTool VERSION 1.0.0 LANGUAGES CXX)

set(PLATFORM_NAME "host" CACHE STRING "Target platform (host, stm32f4)")
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/../thirdparty)
set(ECOLIB_DIR ${CMAKE_SOURCE_DIR}/../ecolib)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Find libusb-1.0
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)



file(GLOB_RECURSE MIDDLEWARE_SRC    
    ${ECOLIB_DIR}/middleware/*.cpp
    ${ECOLIB_DIR}/middleware/*.c
)

file(GLOB_RECURSE CORE_SRC    
    ${ECOLIB}/el/*.cpp
    ${ECOLIB}/el/*.c
)

# Add executable
add_executable(${PROJECT_NAME}
    EcoTool.cpp
    # Header files are listed for IDE visibility, not strictly required for build
    Usbxch.cpp
    Serial.hpp
    ${MIDDLEWARE_SRC}
    ${CORE_SRC}
)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
# Link libusb and threads
target_link_libraries(${PROJECT_NAME}
    ${LIBUSB_LIBRARIES}
    Threads::Threads
    OpenGL::GL 
    glfw
)

# Include directories for libusb
target_include_directories(${PROJECT_NAME} PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ECOLIB_DIR}/middleware
    ${ECOLIB}/el
)

# Compiler flags (optional - remove if not needed)
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -O2
)

# For debugging, uncomment this:
# target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG)

# Install (optional - remove if not needed)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)