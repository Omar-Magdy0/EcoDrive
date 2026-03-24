#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f401xc.h"

#include "eldriver_conf.h"
#include "eldriver_mc3p.h"


#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))
#define Q15_HALF        16384   // 0.5 × 32768
#define Q15_SQRT3_BY_2  28378   // 0.8660254 × 32768

#define CC1E_MASK       (TIM_CCER_CC1E)
#define CC2E_MASK       (TIM_CCER_CC2E)
#define CC3E_MASK       (TIM_CCER_CC3E)
#define CC1NE_MASK      (TIM_CCER_CC1NE)
#define CC2NE_MASK      (TIM_CCER_CC2NE)
#define CC3NE_MASK      (TIM_CCER_CC3NE)

#define OC1M_MASK       (0b111 << TIM_CCMR1_OC1M_Pos)
#define OC1M_PWM        (0b110 << TIM_CCMR1_OC1M_Pos)
#define OC1M_ACTIVE     (0b101 << TIM_CCMR1_OC1M_Pos)
#define OC1M_INACTIVE   (0b100 << TIM_CCMR1_OC1M_Pos)

#define OC2M_MASK       (0b111 << TIM_CCMR1_OC2M_Pos)
#define OC2M_PWM        (0b110 << TIM_CCMR1_OC2M_Pos)
#define OC2M_ACTIVE     (0b101 << TIM_CCMR1_OC2M_Pos)
#define OC2M_INACTIVE   (0b100 << TIM_CCMR1_OC2M_Pos)

#define OC3M_MASK       (0b111 << TIM_CCMR2_OC3M_Pos)
#define OC3M_PWM        (0b110 << TIM_CCMR2_OC3M_Pos)
#define OC3M_ACTIVE     (0b101 << TIM_CCMR2_OC3M_Pos)
#define OC3M_INACTIVE   (0b100 << TIM_CCMR2_OC3M_Pos)

#ifndef ELDRIVER_MC3P_CS
#define ELDRIVER_MC3P_CS ELDRIVER_MC3P_CS_NONE
#endif

#if ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_TRIPLE_SHUNT
#pragma message "CURRENT SENSOR : TRIPLE SHUNT"
#elif ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_DOUBLE_SHUNT
#pragma message "CURRENT SENSOR : DOUBLE SHUNT"
#elif ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_SINGLE_SHUNT
#pragma message "CURRENT SENSOR : SINGLE SHUNT"
#elif ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_NONE
#pragma message "CURRENT SENSOR : NONE"
#elif ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_INLINE
#pragma message "CURRENT SENSOR : INLINE"
#endif

//================================================================
// MCU Family Dependent LAYER GLOBALS and Definitions
//================================================================

//MCU LINE SPECIFIC DEFINES
#define VREFIN_CAL_ADDR (uint16_t *)0x1FFF7A2A
#define VREFIN_CAL_VOLTAGE 3.3f

#ifdef ELDRIVER_MC3P_EXTERNAL_VREF
const float externalRefVoltage_V = ELDRIVER_MC3P_VREFEXT_VOLTAGE;
#else
const float externalRefVoltage_V = 0;
#endif


//======================================================
//DMA static definitions for ADC
uint16_t mc3p_bg_data[2][ELDRIVER_MC3P_BG_CHANNELS];
float mc3p_bg_data_V[ELDRIVER_MC3P_BG_CHANNELS];
static volatile uint8_t mc3p_bg_active_buffer = 0; // 0 or 1
volatile uint8_t isReady = 0;


void mc3p_adc_svm_update(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector);
void mc3p_adc_trap_update(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector);
void mc3p_adc_mode(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector);
void mc3p_offset_calibration(eldriver_mc3p_t *h);

//=========================================
// Adc Single Sample Function
//==========================================
uint16_t adc1_Sample_Single_Channel_Temporary(uint32_t channel)
{
    // Save current configuration
    uint32_t old_sqr1 = ADC1->SQR1;
    uint32_t old_sqr2 = ADC1->SQR2;
    uint32_t old_sqr3 = ADC1->SQR3;
    
    // Configure for single conversion
    // Configure for single conversion USING LL FUNCTION
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE); // 1 conversion
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);   // Handles conversion 
    
    // Start conversion
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1));
    uint16_t result = LL_ADC_REG_ReadConversionData12(ADC1);
    
    // Restore original configuration
    ADC1->SQR1 = old_sqr1;
    ADC1->SQR2 = old_sqr2;
    ADC1->SQR3 = old_sqr3;
    
    return result;
}

float mc3p_adc_read_single(eldriver_mc3p_t *h, uint32_t channel)
{
  uint16_t readVal = adc1_Sample_Single_Channel_Temporary(channel);
  return ((float)(readVal)/( 1<< ELDRIVER_MC3P_ADCRES)) * h->adc_ref_V;
}

//=========================================
// AdcCalibrate Function
//==========================================
void mc3p_adc_calibrate(eldriver_mc3p_t *h)
{
    //Enable VREFINT and internalTEMP sensor
    ADC->CCR |= ADC_CCR_TSVREFE_Msk;
    #ifndef ELDRIVER_MC3P_VREFEXT

    // Wait for internal references to stabilize (IMPORTANT!)
    volatile uint32_t wait = 10000;
    while(wait){wait = wait - 1;};

    // Set longer sampling time for VREFINT
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);

    // Sample VREFINT multiple times for accuracy
    uint32_t sum = 0;
    const uint8_t num_samples = 30;

    //Discard first sample

    adc1_Sample_Single_Channel_Temporary(LL_ADC_CHANNEL_VREFINT);
    for (int i = 0; i < num_samples; i++) {
        sum += adc1_Sample_Single_Channel_Temporary(LL_ADC_CHANNEL_VREFINT);
    }
    uint32_t inRef_div_adcRef = sum / num_samples;

    h->internal_ref_V = (*VREFIN_CAL_ADDR / (float)((1<<ELDRIVER_MC3P_ADCRES) - 1) ) * VREFIN_CAL_VOLTAGE;

    // Calculate actual ADC reference voltage (your supply voltage)
    (h->adc_ref_V) = h->internal_ref_V * ((1<<ELDRIVER_MC3P_ADCRES) - 1) / (float)inRef_div_adcRef;
    #else 

    adc_ref_V = externalRefVoltage_V;
    #endif

    (h->adc_to_uV) = (float)(h->adc_ref_V * 1000000 ) / (float)((1<<ELDRIVER_MC3P_ADCRES) - 1);
}


void mc3p_adc_init(eldriver_mc3p_t *h)
{
        __HAL_RCC_ADC1_CLK_ENABLE();
    LL_ADC_CommonInitTypeDef adc1CommonInitStruct = {
        .CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2
        #ifdef LL_ADC_MULTI_INDEPENDENT
        .Multimode = LL_ADC_MULTI_INDEPENDENT,
        .MultiDMATransfer = LL_ADC_MULTI_REG_DMA_EACH_ADC,
        .MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSAMPLINGDELAY_5CYCLES
        #endif
    };

    LL_ADC_InitTypeDef adc1InitStruct = {
        .Resolution = LL_ADC_RESOLUTION_12B,
        .DataAlignment = LL_ADC_DATA_ALIGN_RIGHT,
        .SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE
    };

    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &adc1CommonInitStruct);
    LL_ADC_Init(ADC1, &adc1InitStruct);

    // Enable ADC
    if (LL_ADC_IsEnabled(ADC1) == 0) 
    {
        LL_ADC_Enable(ADC1);
    }

    //Calibrate Adc
    mc3p_adc_calibrate(h);

    // Initialize ADC regular group for slow measurements
    LL_ADC_REG_InitTypeDef adc1RegInitStruct = {    
        .TriggerSource = LL_ADC_REG_TRIG_SOFTWARE,
        .SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS,
        .SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_3RANKS,
        .ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS,
        .DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED
    };
    LL_ADC_REG_Init(ADC1, &adc1RegInitStruct);
    

    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_56CYCLES);
    
    // Initialize ADC injected group for time-critical measurements
    LL_ADC_INJ_InitTypeDef adc1InjInitStruct = {
        .TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
        .SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS,
        .SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE,
        .TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT
    };                                                      

    // Set sampling times for all PWM Scan channels (injected group)
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_VSBUS_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_28CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_VSU_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_28CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_VSV_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_28CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_VSW_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_28CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_CSU_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_CSV_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, ELDRIVER_MC3P_CSW_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_15CYCLES);

    LL_ADC_INJ_Init(ADC1, &adc1InjInitStruct);
    //ADC CONFIGURATIONS AND LINKAGE TO TIM1 TRGO
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    
    //## Configure use TIM1 CH4 for adc triggering (we exlusively use CH4 for adc triggering)
    //==================================================================
    // USE CHANNEL 4 Exlusively FOR Injected Adc channel (adcPWM Scan)
    //==================================================================
    LL_TIM_OC_InitTypeDef ch4_tim1_OCInit = 
    {
        .OCMode = LL_TIM_OCMODE_PWM1,           // Change to PWM mode
        .OCState = LL_TIM_OCSTATE_ENABLE,       // Enable the output
        .CompareValue = (LL_TIM_GetAutoReload(TIM1) + 1) / 2, // Mid-point for center-aligned
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,   // Rising edge at compare match
        .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
    };
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &ch4_tim1_OCInit);
 
    // Set as trigger output (for ADC synchronization)
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);

    // Enable ADC interrupts
    LL_ADC_EnableIT_JEOS(ADC1);  // Injected sequence end interrupt  
}


#include <stm32f4xx_ll_gpio.h>
#define MC3P_ADC_GpioInit(port, pin)do{\
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};\
    GPIO_InitStruct.Pin = pin;\
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;\
    LL_GPIO_Init(port, &GPIO_InitStruct);\
}while(0)

#define MC3P_PWM_GpioInit(port, pin, active)do{\
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};\
    GPIO_InitStruct.Pin = pin;\
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;\
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;\
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;\
    GPIO_InitStruct.Pull = active?LL_GPIO_PULL_DOWN : LL_GPIO_PULL_UP;\
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;\
    LL_GPIO_Init(port, &GPIO_InitStruct);\
}while(0)


void mc3p_gpio_init()
{     
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_UH_PORT, ELDRIVER_MC3P_UH_PIN, ELDRIVER_MC3P_HIN_ACTIVE);
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_VH_PORT, ELDRIVER_MC3P_VH_PIN, ELDRIVER_MC3P_HIN_ACTIVE);
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_WH_PORT, ELDRIVER_MC3P_WH_PIN, ELDRIVER_MC3P_HIN_ACTIVE);
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_UL_PORT, ELDRIVER_MC3P_UL_PIN, ELDRIVER_MC3P_LIN_ACTIVE);
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_VL_PORT, ELDRIVER_MC3P_VL_PIN, ELDRIVER_MC3P_LIN_ACTIVE);
    MC3P_PWM_GpioInit(ELDRIVER_MC3P_WL_PORT, ELDRIVER_MC3P_WL_PIN, ELDRIVER_MC3P_LIN_ACTIVE);
    //Initialize adc pins
    MC3P_ADC_GpioInit(ELDRIVER_MC3P_VSBUS_PORT, ELDRIVER_MC3P_VSBUS_PIN);
    MC3P_ADC_GpioInit(ELDRIVER_MC3P_VSU_PORT, ELDRIVER_MC3P_VSU_PIN);
    MC3P_ADC_GpioInit(ELDRIVER_MC3P_VSV_PORT, ELDRIVER_MC3P_VSV_PIN);
    MC3P_ADC_GpioInit(ELDRIVER_MC3P_VSW_PORT, ELDRIVER_MC3P_VSW_PIN);
    
    // Optional: External VREF if used
    #ifdef ELDRIVER_MC3P_EXTERNAL_VREF

    #endif
}

//================================================================================================================
// DMA Initialization
//================================================================================================================
void mc3p_dma_init()
{
    // Enable DMA2 clock (ADC1 is connected to DMA2 on STM32F4)
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    // =================================================================
    // DMA for ADC1 Regular Scan (Slow Monitoring Channels)
    // =================================================================
    
    // Configure DMA channel first (ADC1 uses Channel 0 for Stream 0)
    LL_DMA_SetChannelSelection(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, MC3P_DMA_CHANNEL);
    
    // Configure DMA parameters for ADC1 regular conversions
    LL_DMA_SetDataTransferDirection(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM);
    
    // Set peripheral address (ADC1 data register)
    LL_DMA_SetPeriphAddress(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, (uint32_t)&(ADC1->DR));
    
    // Set memory address (double buffer start)
    LL_DMA_SetMemoryAddress(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, (uint32_t)mc3p_bg_data[0]);
    
    // Set data length (total samples across both buffers)
    LL_DMA_SetDataLength(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM, ELDRIVER_MC3P_BG_CHANNELS);

    // Enable DMA transfer complete and half transfer interrupts
    LL_DMA_EnableIT_TC(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM);
    LL_DMA_EnableIT_HT(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM);
    
    // Enable DMA stream
    LL_DMA_EnableStream(MC3P_DMA_INSTANCE, MC3P_DMA_BACKGROUND_STREAM);
}


//================================================================================================================
// Interrupt Initialization
//================================================================================================================
void mc3p_interrupt_init(){
    // Regular scan DMA interrupt - use LL macros for portability
    NVIC_SetPriority(DMA2_Stream0_IRQn, 5);  // Medium priority for slow monitoring
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // ADC Injected End of Conversion (JEOC) interrupt - Higher priority for time-critical measurements
    NVIC_SetPriority(ADC_IRQn, 0);           // Higher priority than DMA for PWM-synchronized measurements
    NVIC_EnableIRQ(ADC_IRQn);
    
    // Enable JEOS (Injected End Of Sequence) interrupt in ADC
    LL_ADC_EnableIT_JEOS(ADC1);
}


static void mc3p_tim1_init(eldriver_mc3p_t *h){
    //ENABLE Peripheral CLOCK
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    //===================================================== 
    // BASIC TIMER CONFIGURATION
    //=====================================================
    LL_TIM_InitTypeDef tim1Init =
    {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_CENTER_UP,
        .Autoreload = __LL_TIM_CALC_ARR(HAL_RCC_GetPCLK2Freq(), 1, h->config.pwm_Hz),
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetUpdateSource(TIM1, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_Init(TIM1, &tim1Init);
    //===================================================== 
    // ADVANCED TIMER CONFIGURATION (BREAK AND DEADTIME)
    //=====================================================
    LL_TIM_BDTR_InitTypeDef tim1BDTRInit = {
        .OSSRState = LL_TIM_OSSR_ENABLE,
        .OSSIState = LL_TIM_OSSI_DISABLE,
        .LockLevel = LL_TIM_LOCKLEVEL_OFF,
        .DeadTime = (uint8_t)(__LL_TIM_CALC_DEADTIME(HAL_RCC_GetPCLK2Freq(), LL_TIM_GetClockDivision(TIM1), h->config.deadtime_nS)),
        .BreakState = LL_TIM_BREAK_DISABLE,
        .BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH,
        .AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE
    };
    LL_TIM_BDTR_Init(TIM1, &tim1BDTRInit);
    //===================================================== 
    // IDENTICAL OUTPUT CHANNEL CONFIGURATION ON CHANNEL 1,2,3
    //=====================================================
    LL_TIM_OC_InitTypeDef ch1ch2ch3_tim1_OCInit = 
    {
        .OCMode = LL_TIM_OCMODE_INACTIVE,
        .OCState = LL_TIM_OCSTATE_ENABLE,
        .OCNState = LL_TIM_OCSTATE_DISABLE,
        .CompareValue = 0,
        .OCPolarity = ELDRIVER_MC3P_HIN_ACTIVE ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
        .OCNPolarity = ELDRIVER_MC3P_LIN_ACTIVE ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
        .OCIdleState = ELDRIVER_MC3P_HIN_ACTIVE ? LL_TIM_OCIDLESTATE_LOW: LL_TIM_OCIDLESTATE_HIGH,
        .OCNIdleState = ELDRIVER_MC3P_LIN_ACTIVE ? LL_TIM_OCIDLESTATE_LOW : LL_TIM_OCIDLESTATE_HIGH
    };
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &ch1ch2ch3_tim1_OCInit);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnablePreload(TIM1);
   
   
    //GET TIMER MAX MAX VALUE TO BE USED FOR DUTY CALCULATION
    h->timer_max_q15 = LL_TIM_GetAutoReload(TIM1);
    // Finally enable the timer (Configuration done)
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
}


void eldriver_mc3p_init(eldriver_mc3p_t *h)
{
    mc3p_gpio_init();
    mc3p_tim1_init(h);
    h->duty_max_q15 = (uint16_t)((h->config.duty_max) * 0x7FFF);
    h->duty_min_q15 = (uint16_t)((h->config.duty_min) * 0x7FFF);

    mc3p_adc_init(h);
    mc3p_dma_init();
    h->sector_last = ELDRIVER_MC3P_SECTOR_FLOAT;
    h->mode        = ELDRIVER_MC3P_MODE_NONE;
    mc3p_irq_bind(h);
    mc3p_interrupt_init();
    if(h->offset_calibration)mc3p_offset_calibration(h);
}

//======================================================
// Phase Ouptut functions
//======================================================
void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w)
{
    uint32_t ccer_shadow = TIM1->CCER;
    uint32_t ccmr1_shadow = TIM1->CCMR1;
    uint32_t ccmr2_shadow = TIM1->CCMR2;
    ccer_shadow |= (CC1E_MASK | CC2E_MASK | CC3E_MASK | CC1NE_MASK | CC2NE_MASK | CC3NE_MASK);  // Set all OCxNE & OCxE bits
    ccmr1_shadow &= ~(OC1M_MASK | OC2M_MASK);  // Clear OC1M bits & OC2M bits
    ccmr2_shadow &= ~OC3M_MASK;  // Clear OC3M bits

    //Operations to be done here
    switch(state_u)
    {
        case(ELDRIVER_MC3P_PHASE_FLOAT)  :   {ccmr1_shadow |= OC1M_INACTIVE; ccer_shadow  &= ~(CC1E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_L_ON)   :   {ccmr1_shadow |= OC1M_INACTIVE; break;}
        case(ELDRIVER_MC3P_PHASE_H_ON)   :   {ccmr1_shadow |= OC1M_ACTIVE;   break;}
        case(ELDRIVER_MC3P_PHASE_COMP)   :   {ccmr1_shadow |= OC1M_PWM;      break;}
        case(ELDRIVER_MC3P_PHASE_L_PWM)  :   {ccmr1_shadow |= OC1M_PWM   ; ccer_shadow &=  ~(CC1E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_H_PWM)  :   {ccmr1_shadow |= OC1M_PWM   ; ccer_shadow &=  ~(CC1NE_MASK); break;}
    }
    switch(state_v)
    {        
        case(ELDRIVER_MC3P_PHASE_FLOAT)  :   {ccmr1_shadow |= OC2M_INACTIVE; ccer_shadow  &= ~(CC2E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_L_ON)   :   {ccmr1_shadow |= OC2M_INACTIVE; break;}
        case(ELDRIVER_MC3P_PHASE_H_ON)   :   {ccmr1_shadow |= OC2M_ACTIVE;   break;}
        case(ELDRIVER_MC3P_PHASE_COMP)   :   {ccmr1_shadow |= OC2M_PWM;      break;}
        case(ELDRIVER_MC3P_PHASE_L_PWM)  :   {ccmr1_shadow |= OC2M_PWM   ; ccer_shadow &=  ~(CC2E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_H_PWM)  :   {ccmr1_shadow |= OC2M_PWM   ; ccer_shadow &=  ~(CC2NE_MASK); break;}
    }
    switch(state_w)
    {
        case(ELDRIVER_MC3P_PHASE_FLOAT)  :   {ccmr2_shadow |= OC3M_INACTIVE; ccer_shadow  &= ~(CC3E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_L_ON)   :   {ccmr2_shadow |= OC3M_INACTIVE; break;}
        case(ELDRIVER_MC3P_PHASE_H_ON)   :   {ccmr2_shadow |= OC3M_ACTIVE;   break;}
        case(ELDRIVER_MC3P_PHASE_COMP)   :   {ccmr2_shadow |= OC3M_PWM;      break;}
        case(ELDRIVER_MC3P_PHASE_L_PWM)  :   {ccmr2_shadow |= OC3M_PWM   ; ccer_shadow &=  ~(CC3E_MASK); break;}
        case(ELDRIVER_MC3P_PHASE_H_PWM)  :   {ccmr2_shadow |= OC3M_PWM   ; ccer_shadow &=  ~(CC3NE_MASK); break;}
    }

    TIM1->CCER = ccer_shadow;
    TIM1->CCMR1 = ccmr1_shadow;
    TIM1->CCMR2 = ccmr2_shadow;
    TIM1->EGR |= TIM_EGR_COMG;
}

void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, uint16_t dutyu_q15, uint16_t dutyv_q15, uint16_t dutyw_q15)
{
    uint32_t compare_u = ((uint32_t)SATURATE(dutyu_q15, h->duty_min_q15, h->duty_max_q15) * h->timer_max_q15) >> 15;
    uint32_t compare_v = ((uint32_t)SATURATE(dutyv_q15, h->duty_min_q15, h->duty_max_q15) * h->timer_max_q15) >> 15;
    uint32_t compare_w = ((uint32_t)SATURATE(dutyw_q15, h->duty_min_q15, h->duty_max_q15) * h->timer_max_q15) >> 15;
    TIM1->CCR1 = compare_u;
    TIM1->CCR2 = compare_v;
    TIM1->CCR3 = compare_w;
}

void eldriver_mc3p_write_float(eldriver_mc3p_t *h)
{    
    eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_FLOAT);
    eldriver_mc3p_write_phase_duty(h, 0, 0, 0);
    h->sector_last = ELDRIVER_MC3P_SECTOR_FLOAT;
}



typedef struct {
    eldriver_mc3p_phase_state_t phase_state[3]; // A,B,C: COMP, L_ON, FLOAT
} mc3p_trap_sector_map_t;


static const mc3p_trap_sector_map_t trap_table[6] = {
    [ELDRIVER_MC3P_SECTOR_TRAP1 - 1] = { {ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_FLOAT}},
    [ELDRIVER_MC3P_SECTOR_TRAP2 - 1] = { {ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_L_ON}},
    [ELDRIVER_MC3P_SECTOR_TRAP3 - 1] = { {ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_L_ON}},
    [ELDRIVER_MC3P_SECTOR_TRAP4 - 1] = { {ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_FLOAT}},
    [ELDRIVER_MC3P_SECTOR_TRAP5 - 1] = { {ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_COMP}},
    [ELDRIVER_MC3P_SECTOR_TRAP6 - 1] = { {ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_COMP}},
};

void eldriver_mc3p_write_trap(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15)
{
    if(!IS_TRAP_SECTOR(h->sector_last))
    {
        mc3p_adc_mode(h, sector);
        h->mode        = ELDRIVER_MC3P_MODE_TRAP;
    }
 

    if(sector < ELDRIVER_MC3P_SECTOR_TRAP1 || sector > ELDRIVER_MC3P_SECTOR_TRAP6)
        return;

    const mc3p_trap_sector_map_t *map = &trap_table[sector - 1];

    // Only update phase state & ADC if sector changed
    if(h->sector_last != sector)
    {
        eldriver_mc3p_write_phase_state(h, map->phase_state[0], map->phase_state[1], map->phase_state[2]);
        mc3p_adc_trap_update(h, sector);
        h->sector_last = sector;
    }
    eldriver_mc3p_write_phase_duty(h, duty_q15, duty_q15, duty_q15);
}

void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15) 
{
    int32_t vmax, vmin, voff;
    if(!IS_SVM_SECTOR(h->sector_last))
    {
        eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP);
        h->mode        = ELDRIVER_MC3P_MODE_SVM;
    }

    /* αβ → phase (normalized Q15) */
    h->dutyu_q15 = alpha_q15;

    h->dutyv_q15 = (-(int32_t)Q15_HALF * alpha_q15
         + (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    h->dutyw_q15 = (-(int32_t)Q15_HALF * alpha_q15
         - (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    /* SVPWM zero-sequence injection */
    uint8_t b0 = (h->dutyu_q15 >= 0);
    uint8_t b1 = (h->dutyv_q15 >= 0);
    uint8_t b2 = (h->dutyw_q15 >= 0);

    // 3-bit code
    uint8_t code = (b2 << 2) | (b1 << 1) | b0;
    eldriver_mc3p_sector_t sector;
    switch(code)
    {
        case 0b001: sector = 1; break;
        case 0b011: sector = 2; break;
        case 0b010: sector = 3; break;
        case 0b110: sector = 4; break;
        case 0b100: sector = 5; break;
        case 0b101: sector = 6; break;
        default: sector = 0; break; // should not happen
    }
     /* SVPWM zero-sequence injection */
    vmax = h->dutyu_q15;
    if (h->dutyv_q15 > vmax) vmax = h->dutyv_q15;
    if (h->dutyw_q15 > vmax) vmax = h->dutyw_q15;

    vmin = h->dutyu_q15;
    if (h->dutyv_q15 < vmin) vmin = h->dutyv_q15;
    if (h->dutyw_q15 < vmin) vmin = h->dutyw_q15;

    voff = (vmax + vmin) >> 1;

    h->dutyu_q15 -= voff;
    h->dutyv_q15 -= voff;
    h->dutyw_q15 -= voff;
    mc3p_adc_svm_update(h, sector);
    eldriver_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
    h->sector_last = sector;
}

//==============================================
// Analog Reading functions
//==============================================
void mc3p_adc_mode(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector)
{
    uint8_t is_svm = IS_SVM_SECTOR(sector);
    uint8_t is_trap = IS_TRAP_SECTOR(sector);
    if(is_svm)
    {
        LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, ELDRIVER_MC3P_VSBUS_ADC_CHANNEL);
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, ELDRIVER_MC3P_CSU_ADC_CHANNEL);
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, ELDRIVER_MC3P_CSV_ADC_CHANNEL);
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_4, ELDRIVER_MC3P_CSW_ADC_CHANNEL);
        h->sync_rank_scale[0] = ELDRIVER_MC3P_VSBUS;
        h->sync_rank_scale[1] = ELDRIVER_MC3P_CSU;
        h->sync_rank_scale[2] = ELDRIVER_MC3P_CSV;
        h->sync_rank_scale[3] = ELDRIVER_MC3P_CSW;
    }else if(is_trap)
    {
        LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS);
        LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, ELDRIVER_MC3P_VSBUS_ADC_CHANNEL);
        h->sync_rank_scale[0] = ELDRIVER_MC3P_VSBUS;
    }
}

void mc3p_adc_svm_update(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector)
{
    
}


typedef struct {
    uint32_t rank2_channel;
    uint32_t rank3_channel;
    uint8_t  rank2_scale_idx;
    uint8_t  rank3_scale_idx;
} mc3p_adc_map_t;

static const mc3p_adc_map_t adc_trap_table[6] = {
    [ELDRIVER_MC3P_SECTOR_TRAP1 - 1] = { ELDRIVER_MC3P_VSW_ADC_CHANNEL, ELDRIVER_MC3P_CSV_ADC_CHANNEL, ELDRIVER_MC3P_VSW, ELDRIVER_MC3P_CSV},
    [ELDRIVER_MC3P_SECTOR_TRAP2 - 1] = { ELDRIVER_MC3P_VSV_ADC_CHANNEL, ELDRIVER_MC3P_CSW_ADC_CHANNEL, ELDRIVER_MC3P_VSV, ELDRIVER_MC3P_CSW},
    [ELDRIVER_MC3P_SECTOR_TRAP3 - 1] = { ELDRIVER_MC3P_VSU_ADC_CHANNEL, ELDRIVER_MC3P_CSW_ADC_CHANNEL, ELDRIVER_MC3P_VSU, ELDRIVER_MC3P_CSW},
    [ELDRIVER_MC3P_SECTOR_TRAP4 - 1] = { ELDRIVER_MC3P_VSW_ADC_CHANNEL, ELDRIVER_MC3P_CSU_ADC_CHANNEL, ELDRIVER_MC3P_VSW, ELDRIVER_MC3P_CSU},
    [ELDRIVER_MC3P_SECTOR_TRAP5 - 1] = { ELDRIVER_MC3P_VSV_ADC_CHANNEL, ELDRIVER_MC3P_CSU_ADC_CHANNEL, ELDRIVER_MC3P_VSV, ELDRIVER_MC3P_CSU},
    [ELDRIVER_MC3P_SECTOR_TRAP6 - 1] = { ELDRIVER_MC3P_VSU_ADC_CHANNEL, ELDRIVER_MC3P_CSV_ADC_CHANNEL, ELDRIVER_MC3P_VSU, ELDRIVER_MC3P_CSV},
};

void mc3p_adc_trap_update(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector)
{
    const mc3p_adc_map_t *map = &adc_trap_table[sector - 1];
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, map->rank2_channel);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, map->rank3_channel);
}

#include <string.h>
void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void* data)
{
    uint8_t is_svm = IS_SVM_SECTOR(h->sector_last);
    uint8_t is_trap = IS_TRAP_SECTOR(h->sector_last);
    if(is_svm)
    {
        ((eldriver_mc3p_trap_data_t *)(data))->vbus_q31  = (h->sync_scale_q31[h->sync_rank_scale[0]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)) + h->sync_scale_q31[h->sync_rank_scale[0]][1]);
        ((eldriver_mc3p_svm_data_t *)(data))->cu_q31     = (h->sync_scale_q31[h->sync_rank_scale[1]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)) + h->sync_scale_q31[h->sync_rank_scale[1]][1]);
        ((eldriver_mc3p_svm_data_t *)(data))->cv_q31     = (h->sync_scale_q31[h->sync_rank_scale[2]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2)) + h->sync_scale_q31[h->sync_rank_scale[2]][1]);  
        ((eldriver_mc3p_svm_data_t *)(data))->cw_q31     = (h->sync_scale_q31[h->sync_rank_scale[3]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3)) + h->sync_scale_q31[h->sync_rank_scale[3]][1]);        
    }
    else if (is_trap)
    {
        ((eldriver_mc3p_trap_data_t *)(data))->vbus_q31  = (h->sync_scale_q31[h->sync_rank_scale[0]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)) + h->sync_scale_q31[h->sync_rank_scale[0]][1]);
        ((eldriver_mc3p_trap_data_t *)(data))->vbemf_q31 = (h->sync_scale_q31[h->sync_rank_scale[1]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2)) + h->sync_scale_q31[h->sync_rank_scale[1]][1]);
        ((eldriver_mc3p_trap_data_t *)(data))->cbus_q31  = (h->sync_scale_q31[h->sync_rank_scale[2]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3)) + h->sync_scale_q31[h->sync_rank_scale[2]][1]);
    }
    else{
        ((eldriver_mc3p_trap_data_t *)(data))->vbus_q31  = (h->sync_scale_q31[h->sync_rank_scale[0]][0] * (LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1)) + h->sync_scale_q31[h->sync_rank_scale[0]][1]);
    }
}

uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float* scanData)
{
    uint8_t readBuffer = 1 - mc3p_bg_active_buffer; // Always read from inactive buffer
    for(int i = 0; i < ELDRIVER_MC3P_BG_CHANNELS; i++)
    {
        mc3p_bg_data_V[i] = (h->adc_to_uV * (mc3p_bg_data[readBuffer][i]) ) / 1000000.0;
    }
    memcpy(scanData, mc3p_bg_data_V, ELDRIVER_MC3P_BG_CHANNELS * sizeof(float));
    isReady = 0;
    return ELDRIVER_MC3P_BG_CHANNELS;
}


void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h)
{
    isReady = 0;
    LL_ADC_REG_StartConversionSWStart(ADC1);
}

uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h)
{
    return ELDRIVER_MC3P_BG_CHANNELS;
}

uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h)
{
    return isReady;
}


void DMA2_Stream0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_HT0(MC3P_DMA_INSTANCE)) {
        LL_DMA_ClearFlag_HT0(MC3P_DMA_INSTANCE);
        // Half transfer complete - First Half samples Done
        mc3p_bg_active_buffer = 1;
        isReady = 1;
    }
    
    if (LL_DMA_IsActiveFlag_TC0(MC3P_DMA_INSTANCE)) {
        LL_DMA_ClearFlag_TC0(MC3P_DMA_INSTANCE);
        // Full transfer complete - Second Half Samples Done
        mc3p_bg_active_buffer = 0;
        isReady = 1;
    }
}


void eldriver_mc3p_setGain(eldriver_mc3p_t *h,  eldriver_mc3p_sync s, float gain)
{
    float scale = (s >= ELDRIVER_MC3P_CSU && s <= ELDRIVER_MC3P_CSW)? ELDRIVER_MC3P_CS_SCALE : ELDRIVER_MC3P_VS_SCALE;
    h->sync_scale_q31[s][0] = ((gain * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * scale));
}

void mc3p_offset_calibration(eldriver_mc3p_t *h)
{
    // Set all phases to Loww (0,0,0)
    // Set mode to calibration
    h->mode = ELDRIVER_MC3P_MODE_CALIB;
    eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_L_ON);
    eldriver_mc3p_write_phase_duty(h, 0, 0, 0);
    // Read current sensor values (the reading is your offset) (make the injected sequence suitable)
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, ELDRIVER_MC3P_VSBUS_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, ELDRIVER_MC3P_CSU_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, ELDRIVER_MC3P_CSV_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_4, ELDRIVER_MC3P_CSW_ADC_CHANNEL);
    for(int i = 0; i < 4; i++)h->offset_calibration_sum[i] = 0;
    h->offset_calibration_remaining_samples = ELDRIVER_MC3P_OFFSET_CALIBRATION_SAMPLES;
    while(h->offset_calibration_remaining_samples);
    // Now we have the offsets
    for(int i = 0; i < 4; i++)h->offset_calibration_sum[i] = h->offset_calibration_sum[i]/ELDRIVER_MC3P_OFFSET_CALIBRATION_SAMPLES;
    //assign the averaged adc value after Q31 scaling
    h->sync_scale_q31[ELDRIVER_MC3P_CSU][1] =  ((h->offset_calibration_sum[1] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_CS_SCALE));
    h->sync_scale_q31[ELDRIVER_MC3P_CSV][1] =  ((h->offset_calibration_sum[2] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_CS_SCALE));
    h->sync_scale_q31[ELDRIVER_MC3P_CSW][1] =  ((h->offset_calibration_sum[3] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_CS_SCALE));

    // Read phase voltage sensor values (the reading is your offset) (make the injected sequence suitable)
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, ELDRIVER_MC3P_VSBUS_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, ELDRIVER_MC3P_VSU_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, ELDRIVER_MC3P_VSV_ADC_CHANNEL);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_4, ELDRIVER_MC3P_VSW_ADC_CHANNEL);
    for(int i = 0; i < 4; i++)h->offset_calibration_sum[i] = 0;
    h->offset_calibration_remaining_samples = ELDRIVER_MC3P_OFFSET_CALIBRATION_SAMPLES;
    while(h->offset_calibration_remaining_samples);
    // Now we have the offsets
    for(int i = 0; i < 4; i++)h->offset_calibration_sum[i] = h->offset_calibration_sum[i]/ELDRIVER_MC3P_OFFSET_CALIBRATION_SAMPLES;
    //assign the averaged adc value after Q31 scaling
    h->sync_scale_q31[ELDRIVER_MC3P_VSU][1] =  ((h->offset_calibration_sum[1] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_VS_SCALE));
    h->sync_scale_q31[ELDRIVER_MC3P_VSV][1] =  ((h->offset_calibration_sum[2] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_VS_SCALE));
    h->sync_scale_q31[ELDRIVER_MC3P_VSW][1] =  ((h->offset_calibration_sum[3] * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * ELDRIVER_MC3P_VS_SCALE));

    //Set phases to High-Z and call it a day
    h->mode = ELDRIVER_MC3P_MODE_NONE;
    eldriver_mc3p_write_float(h);
}

void INTERNAL_mc3p_ADC_JEOS_IRQ(eldriver_mc3p_t *h)
{
    switch (h->mode)
    {
    case ELDRIVER_MC3P_MODE_CALIB:
    if(h->offset_calibration_remaining_samples){
        h->offset_calibration_sum[0] += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        h->offset_calibration_sum[1] += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
        h->offset_calibration_sum[2] += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
        h->offset_calibration_sum[3] += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_4);
        h->offset_calibration_remaining_samples--;
    }
        break;
    
    default:
        break;
    }
}