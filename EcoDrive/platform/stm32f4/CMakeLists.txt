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
set(HAL_CONFIG_DIR ${THIRDPARTY_DIR}/stm32/f4)
set(STM32CMX_DIR ${THIRDPARTY_DIR}/stm32)
set(STM32F4xx_HAL_DRIVER_DIR ${THIRDPARTY_DIR}/stm32/f4/STM32F4xx_HAL_Driver)
set(CMSIS_DEVICE_DIR ${THIRDPARTY_DIR}/CMSIS/Device/ST/STM32F4xx/Include)
set(CMSIS_CORE_DIR ${THIRDPARTY_DIR}/CMSIS)
set(CMSIS_DSP_DIR ${THIRDPARTY_DIR}/CMSIS-DSP)
set(CMSIS_DSP_PRIVATE_INCLUDES
    ${CMSIS_DSP_DIR}/PrivateInclude
    ${CMSIS_DSP_DIR}/Include/dsp  # Sometimes needed
)
set(USBD_DIR ${THIRDPARTY_DIR}/stm32/usbd)


# Common include directories for all STM32 targets
set(STM32_COMMON_INCLUDES
    ${CMSIS_CORE_DIR}/Include
    ${CMSIS_CORE_DIR}/Core/Include
    ${CMSIS_DSP_PRIVATE_INCLUDES}
    ${STM32F4xx_HAL_DRIVER_DIR}/Inc
    ${STM32F4xx_HAL_DRIVER_DIR}/Inc/Legacy  # Important for HAL/LL compatibility
    ${CMSIS_DEVICE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${HAL_CONFIG_DIR}
    ${USBD_DIR}/Core/Inc
    ${USBD_DIR}/Class/XCH/Inc
    ${USBD_DIR}/App
    ${STM32CMX_DIR}/f4/Target
    ${ELDRIVER_DIR}/stm32f4
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


set(Platform_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.c
)

set(STM32CMX_Src
    ${STM32CMX_DIR}/syscalls.c
    ${STM32CMX_DIR}/sysmem.c
    ${STM32CMX_DIR}/f4/stm32f4xx_it.c
    ${STM32CMX_DIR}/f4/toolchain/startup_stm32f401xc.s
    ${STM32CMX_DIR}/f4/system_stm32f4xx.c
)

set(USBD_Src
    ${USBD_DIR}/Core/Src/usbd_core.c
    ${USBD_DIR}/Core/Src/usbd_ctlreq.c
    ${USBD_DIR}/Core/Src/usbd_ioreq.c
    ${USBD_DIR}/Class/XCH/Src/usbd_xch.c
    ${USBD_DIR}/App/usbd_desc.c
    ${USBD_DIR}/App/usb_device.c
    ${STM32CMX_DIR}/f4/Target/usbd_conf.c
)

set(CMSISCORE ${THIRDPARTY_DIR}/CMSIS)
add_subdirectory(${CMSIS_DSP_DIR}/Source ${CMAKE_CURRENT_BINARY_DIR}/cmsis_dsp_stm32f4)

# Driver core sources are NOT added to the static library to avoid linker archive issues
# where weak startup symbols prevent strong handler definitions from being linked.
# Instead, they are linked directly from the parent scope (see PLATFORM_DRIVER_CORE_SOURCES).
file(GLOB_RECURSE Driver_Core_Src
    ${ELDRIVER_DIR}/stm32f4/*.cpp
    ${ELDRIVER_DIR}/stm32f4/*.c
)

add_library(stm32f4_platform STATIC)
target_sources(stm32f4_platform PRIVATE
    ${STM32CMX_Src}
    ${STM32_Drivers_Src}
    ${USBD_Src}
)

target_include_directories(stm32f4_platform PUBLIC ${STM32_COMMON_INCLUDES} ${CMSIS_DSP_DIR}/Include )
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
    ${CMSISCORE}/Include
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
