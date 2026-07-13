cmake_minimum_required(VERSION 3.20)

# Host platform module used by top-level CMake via add_subdirectory(platform/host).

set(DEVICE_DEFINITIONS
    USE_FULL_ASSERT
)

set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/../thirdparty)
set(ELDRIVER_DIR ${CMAKE_SOURCE_DIR}/../eldriver)


set(CMSIS_DSP_DIR ${THIRDPARTY_DIR}/CMSIS-DSP)
set(CMSISCORE ${THIRDPARTY_DIR}/CMSIS)
set(IMGUI_DIR ${THIRDPARTY_DIR}/imgui)
set(IMPLOT_DIR ${THIRDPARTY_DIR}/implot)
set(IMGUIFD_DIR ${THIRDPARTY_DIR}/ImGuiFileDialog)

set(HOST_COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${IMGUIFD_DIR}
    ${EIGEN_DIR}
    ${ELDRIVER_DIR}/host
)

set(IMGUI_SRC  
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUIFD_DIR}/ImGuiFileDialog.cpp
)

set(Platform_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.cpp
)

file(GLOB_RECURSE Driver_Core_Src
    ${ELDRIVER_DIR}/host/*.cpp
    ${ELDRIVER_DIR}/host/*.c
)

find_package(Threads REQUIRED)
add_subdirectory(${CMSIS_DSP_DIR}/Source ${CMAKE_CURRENT_BINARY_DIR}/cmsis_dsp_host)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

add_library(host_platform STATIC)
target_sources(host_platform PRIVATE
    ${IMGUI_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/TCPServer.cpp
)

target_include_directories(host_platform PUBLIC ${HOST_COMMON_INCLUDES})
target_compile_definitions(host_platform PUBLIC
    ${DEVICE_DEFINITIONS}
    projCOVERAGE_TEST=0
    projENABLE_TRACING=0
)

target_link_libraries(host_platform PUBLIC ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES} Threads::Threads CMSISDSP OpenGL::GL glfw)

add_library(host_platform_interface INTERFACE)
target_link_libraries(host_platform_interface INTERFACE
    host_platform
    Threads::Threads
    CMSISDSP
)

target_include_directories(host_platform_interface INTERFACE
    ${HOST_COMMON_INCLUDES}
    ${HDF5_INCLUDE_DIRS}
    ${CMSIS_DSP_DIR}/Include
    ${CMSISCORE}/Include
)
target_compile_definitions(host_platform_interface INTERFACE
    ${DEVICE_DEFINITIONS}
    projCOVERAGE_TEST=0
    projENABLE_TRACING=0
)

# Export variables expected by the parent CMakeLists.txt.
set(PLATFORM_LIB host_platform PARENT_SCOPE)
set(PLATFORM_INTERFACE host_platform_interface PARENT_SCOPE)
set(PLATFORM_INCLUDES ${HOST_COMMON_INCLUDES} ${CMSIS_DSP_DIR}/Include ${CMSISCORE}/Include PARENT_SCOPE)
set(PLATFORM_DEFINITIONS ${DEVICE_DEFINITIONS} PARENT_SCOPE)
set(PLATFORM_SOURCE ${Driver_Core_Src} ${Platform_Src} PARENT_SCOPE)
