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
set(HAL_CONFIG_DIR ${CMAKE_SOURCE_DIR})
set(STM32F4xx_HAL_DRIVER_DIR ${CMAKE_SOURCE_DIR}/thirdparty/stm32/STM32F4xx_HAL_Driver)
set(CMSIS_DEVICE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/CMSIS/Device/ST/STM32F4xx/Include)
set(CMSIS_CORE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/CMSIS)
set(CMSIS_DSP_DIR ${CMAKE_SOURCE_DIR}/thirdparty/CMSIS-DSP)
set(TINYUSB_DIR ${CMAKE_SOURCE_DIR}/thirdparty/tinyusb)
set(CMSIS_DSP_PRIVATE_INCLUDES
    ${CMSIS_DSP_DIR}/PrivateInclude
    ${CMSIS_DSP_DIR}/Include/dsp  # Sometimes needed
)


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
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f401xc.s
    ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c
)

set(TinyUsb_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_desc.c  
    ${TINYUSB_DIR}/src/tusb.c
	${TINYUSB_DIR}/src/common/tusb_fifo.c
	${TINYUSB_DIR}/src/device/usbd.c
	${TINYUSB_DIR}/src/typec/usbc.c
	${TINYUSB_DIR}/src/class/audio/audio_device.c
	${TINYUSB_DIR}/src/class/cdc/cdc_device.c
	${TINYUSB_DIR}/src/class/dfu/dfu_device.c
	${TINYUSB_DIR}/src/class/dfu/dfu_rt_device.c
	${TINYUSB_DIR}/src/class/hid/hid_device.c
	${TINYUSB_DIR}/src/class/midi/midi_device.c
	${TINYUSB_DIR}/src/class/midi/midi2_device.c
	${TINYUSB_DIR}/src/class/msc/msc_device.c
	${TINYUSB_DIR}/src/class/mtp/mtp_device.c
	${TINYUSB_DIR}/src/class/net/ecm_rndis_device.c
	${TINYUSB_DIR}/src/class/net/ncm_device.c
	${TINYUSB_DIR}/src/class/printer/printer_device.c
	${TINYUSB_DIR}/src/class/usbtmc/usbtmc_device.c
	${TINYUSB_DIR}/src/class/video/video_device.c
	${TINYUSB_DIR}/src/class/vendor/vendor_device.c
    ${TINYUSB_DIR}/src/host/usbh.c
    ${TINYUSB_DIR}/src/host/hub.c
    ${TINYUSB_DIR}/src/class/cdc/cdc_host.c
    ${TINYUSB_DIR}/src/class/hid/hid_host.c
    ${TINYUSB_DIR}/src/class/midi/midi_host.c
    ${TINYUSB_DIR}/src/class/midi/midi2_host.c
    ${TINYUSB_DIR}/src/class/msc/msc_host.c
    ${TINYUSB_DIR}/src/class/vendor/vendor_host.c
    ${TINYUSB_DIR}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ${TINYUSB_DIR}/src/portable/st/stm32_fsdev/fsdev_common.c
    ${TINYUSB_DIR}/src/portable/st/stm32_fsdev/hcd_stm32_fsdev.c
    ${TINYUSB_DIR}/src/portable/st/typec/typec_stm32.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/hcd_dwc2.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dwc2_common.c
)

set(TinyUsb_Includes
    ${TINYUSB_DIR}/src
)

set(CMSISCORE ${CMAKE_SOURCE_DIR}/thirdparty/CMSIS)
add_subdirectory(${CMSIS_DSP_DIR}/Source ${CMAKE_CURRENT_BINARY_DIR}/cmsis_dsp_stm32f4)

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
    ${TinyUsb_Src}
)

target_include_directories(stm32f4_platform PUBLIC ${STM32_COMMON_INCLUDES} ${CMSIS_DSP_DIR}/Include ${TinyUsb_Includes})
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
