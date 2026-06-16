cmake_minimum_required(VERSION 3.20)

# STM32F1 Platform Module - Minimal abstraction layer
# This is a platform interface only - actual HAL/drivers are user-provided

set(DEVICE_DEFINITIONS
    STM32F103xB
)

set(FREERTOS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
)

# Minimal include path (abstraction layer only)
set(STM32_COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/DSP/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
)

# Core platform files only
set(Platform_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f1xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103xb.s
    ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sysmem.c
)

# Eldriver files
set(Eldriver_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_mc3p.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_mc3p_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_hall.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_usbcdc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/eldriver_uart.c
)

# Minimal static library - no HAL/RTOS sources
add_library(stm32f1_platform STATIC)
target_sources(stm32f1_platform PRIVATE
    ${Platform_Src}
    ${Eldriver_Src}
    ${FREERTOS_SRC}
)

target_include_directories(stm32f1_platform PUBLIC ${STM32_COMMON_INCLUDES})
target_compile_definitions(stm32f1_platform PUBLIC ${DEVICE_DEFINITIONS})

# Interface library for export
add_library(stm32f1_platform_interface INTERFACE)
target_link_libraries(stm32f1_platform_interface INTERFACE stm32f1_platform)
target_include_directories(stm32f1_platform_interface INTERFACE ${STM32_COMMON_INCLUDES})
target_compile_definitions(stm32f1_platform_interface INTERFACE ${DEVICE_DEFINITIONS})

# Export for parent CMakeLists
set(PLATFORM_LIB stm32f1_platform PARENT_SCOPE)
set(PLATFORM_INTERFACE stm32f1_platform_interface PARENT_SCOPE)
set(PLATFORM_INCLUDES ${STM32_COMMON_INCLUDES} PARENT_SCOPE)
set(PLATFORM_DEFINITIONS ${DEVICE_DEFINITIONS} PARENT_SCOPE)
set(PLATFORM_SOURCE ${Platform_Src} ${Eldriver_Src} PARENT_SCOPE)
