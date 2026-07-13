#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "eldriver_conf.h"
#include "elcore.h"
#include "eldriver_uart.h"
#include <string.h>

#ifdef ELDRIVER_UART1_ENABLED

//======================================================
//DMA static definitions for UART1 DMA and UART instance
//======================================================
static uint8_t uart1_tx_buffer[ELDRIVER_UART1_TX_BUFFER_SIZE];
static uint8_t uart1_rx_buffer[ELDRIVER_UART1_RX_BUFFER_SIZE];

//NEVER FORGET 
//THAT YOU EVER CHANGE THESE VALUES FOR DMA YOU STILL HAVE TO CHANGE IMPLEMENTATION AND IRQ APPROPIATELY


void uart1_dma_receive_handle();

static void uart1_dma_transmit_complete_callback();

#if UART_I1_DMA_RECEIVE_COMPLETE_CALLBACK_ENABLED
void (*uart1_dma_receive_complete_callback)(void) = nullptr;
#endif 

void uart1_dma_receive_complete_register_callback(void(*func)(void));



void uart1_dmaInit();
void uart1_interruptInit();
void uart1_uartInit(eldriver_uart_handle_t *handle);


//================================================
// Static memory management definitions for USART1
//================================================


// Rx and Tx Buffer management
uint8_t uart1_rx_buf[ELDRIVER_UART1_RX_BUFFER_SIZE];
uint8_t uart1_tx_buf[ELDRIVER_UART1_TX_BUFFER_SIZE];
elcore_rstream_t uart1_rx_stream;
elcore_rstream_t uart1_tx_stream;



//============================================
// HARDWARE INITIALIZATION FUNCTIONS
//============================================
void uart1_dmaInit()
{
    // Enable DMA2 clock
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Enable DMA clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    
    //#######
    // RX DMA Configuration (Circular)
    //#######
    LL_DMA_SetChannelSelection(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, UART1_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    
    LL_DMA_SetPeriphAddress(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, (uint32_t)&USART1->DR);
    
    // Enable RX DMA interrupts
    LL_DMA_EnableIT_TC(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    LL_DMA_EnableIT_HT(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);

    LL_DMA_SetMemoryAddress(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, (uint32_t)uart1_rx_buffer);
    
    //Start the DMA reception on Rx!
    LL_DMA_SetDataLength(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM, ELDRIVER_UART1_RX_BUFFER_SIZE);
    LL_DMA_EnableStream(UART1_DMA_INSTANCE, UART1_DMA_RX_STREAM);
    
    //#######
    // TX DMA Configuration (Normal)
    //#######
    LL_DMA_SetChannelSelection(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, UART1_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
    
    LL_DMA_SetPeriphAddress(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM, (uint32_t)&USART1->DR);
    
    // Enable TX DMA interrupt
    LL_DMA_EnableIT_TC(UART1_DMA_INSTANCE, UART1_DMA_TX_STREAM);
}

void uart1_interruptInit()
{
    // DMA interrupts
    NVIC_SetPriority(DMA2_Stream5_IRQn, UART1_NVIC_PRIORITY);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    
    NVIC_SetPriority(DMA2_Stream7_IRQn, UART1_NVIC_PRIORITY);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    
    // UART interrupt
    NVIC_SetPriority(USART1_IRQn, UART1_NVIC_PRIORITY);
    NVIC_EnableIRQ(USART1_IRQn);
    
}

#include "stm32f4xx_ll_gpio.h"
#define UART1_GpioInit(port, pin)do{\
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);\
    LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_HIGH);\
    LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);\
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_UP);\
    GPIO_SET_AF(port, pin, LL_GPIO_AF_7);\
}while(0)

void uart1_gpio_init()
{
    UART1_GpioInit(GPIOB, );
    UART1_GpioInit(GPIOB, );
}



//===========================================
// FULL INITIALIZATION FUNCTION
//============================================
void eldriver_uart1_init(eldriver_uart_handle_t *handle)
{
    // Initialize GPIO first
    uart1_gpioInit();
    uart1_dmaInit();
    uart1_interruptInit();
    uart1_uartInit(handle);
    // Initialize buffer management variables
}





elcore_rstream_stats_t eldriver_uart1_rx_stats(eldriver_uart_handle_t *handle)
{
    return elcore_rstream_getStats(&uart1_rx_buffer);
}

elcore_rstream_stats_t eldriver_uart1_tx_stats(eldriver_uart_handle_t *handle)
{
    return elcore_rstream_getStats(&uart1_tx_buffer);
}

void eldriver_uart1_resetStats(eldriver_uart_handle_t *handle)
{
    elcore_rstream_reserveWrite(&uart1_rx_buffer);
    elcore_rstream_reserveWrite(&uart1_tx_buffer);
}

//===========================================
// CALLBACK AND INTERRUPT DEFINTIONS
//============================================
// External C callbacks for IRQ Handlers

void DMA2_Stream5_IRQHandler(void) {

}

void DMA2_Stream7_IRQHandler(void) 
{

}




#endif