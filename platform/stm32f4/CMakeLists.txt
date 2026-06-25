cmake_minimum_required(VERSION 3.20)
# Include device-specific CMake configuration

# Set device specific definitions
set(DEVICE_DEFINITIONS
    USE_HAL_DRIVER
    STM32F401xC
    USE_FULL_LL_DRIVER
    #USE_FULL_ASSERT
)

# Path configurations
set(HAL_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(STM32F4xx_HAL_DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver)
set(CMSIS_DEVICE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
set(CMSIS_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)
set(CMSIS_DSP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS-DSP)
set(USBD_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library)
set(USB_CLASS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library)

# Common include directories for all STM32 targets
set(STM32_COMMON_INCLUDES
    ${STM32F4xx_HAL_DRIVER_DIR}/Inc
    ${STM32F4xx_HAL_DRIVER_DIR}/Inc/Legacy  # Important for HAL/LL compatibility
    ${CMSIS_DEVICE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_CORE_DIR}
    ${HAL_CONFIG_DIR}
    ${USBD_LIB_DIR}/Core/Inc
    ${USB_CLASS_DIR}/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_impl
)

# ===== HAL DRIVERS (for complex protocols) =====
set(STM32_Drivers_Src
    # Core HAL
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_usb.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_pcd.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_pcd_ex.c

    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_rcc.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_rcc_ex.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_gpio.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_cortex.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_dma.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_pwr.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_pwr_ex.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_flash.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_flash_ex.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_tim.c     
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_tim_ex.c  
    # Communication HAL (add as needed)
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_uart.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_hal_can.c

        # Core LL
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_utils.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_rcc.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_gpio.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_dma.c
    
    # Motor Control LL (add as needed)
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_tim.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_adc.c
    ${STM32F4xx_HAL_DRIVER_DIR}/Src/stm32f4xx_ll_usart.c
)

set(USB_Device_Library_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

set(Platform_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f401xc.s
    ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c
)


set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)
add_subdirectory(${CMSIS_DSP_DIR}/Source)

# Driver core sources are NOT added to the static library to avoid linker archive issues
# where weak startup symbols prevent strong handler definitions from being linked.
# Instead, they are linked directly from the parent scope (see PLATFORM_DRIVER_CORE_SOURCES).
file(GLOB_RECURSE Driver_Core_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/*.c
)

add_library(stm32f4_platform STATIC)
target_sources(stm32f4_platform PRIVATE
    ${STM32_Drivers_Src}
    ${USB_Device_Library_Src}
)

target_include_directories(stm32f4_platform PUBLIC ${STM32_COMMON_INCLUDES} ${CMSIS_DSP_DIR}/Include)
target_compile_definitions(stm32f4_platform PUBLIC ${DEVICE_DEFINITIONS})

# =============================================
# INTERFACE LIBRARY FOR EXPORT
# =============================================
# Create interface library for easy linking
add_library(stm32f4_platform_interface INTERFACE)
target_link_libraries(stm32f4_platform_interface INTERFACE stm32f4_platform CMSISDSP)
target_include_directories(stm32f4_platform_interface INTERFACE
    ${STM32_COMMON_INCLUDES}
    ${CMSIS_DSP_DIR}/Include
)
target_compile_definitions(stm32f4_platform_interface INTERFACE
    ${DEVICE_DEFINITIONS}
)

# =============================================
# EXPORT VARIABLES FOR PARENT CMakeLists.txt
# =============================================
# These will be available in the parent scope
set(PLATFORM_LIB stm32f4_platform PARENT_SCOPE)
set(PLATFORM_INTERFACE stm32f4_platform_interface PARENT_SCOPE)
set(PLATFORM_INCLUDES ${STM32_COMMON_INCLUDES} PARENT_SCOPE)
set(PLATFORM_DEFINITIONS ${DEVICE_DEFINITIONS} PARENT_SCOPE)
set(PLATFORM_SOURCE ${Driver_Core_Src} ${Platform_Src} PARENT_SCOPE)
