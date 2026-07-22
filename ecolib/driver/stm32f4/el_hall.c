#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "el_hall.h"
#include "eld_conf.h"


#define CLK_FREQ 1000000  // 1 MHz = 1us resolution
#define CLK_TO_US(clk) ((clk) * 1000000ULL / CLK_FREQ)
#define US_TO_CLK(us)(us * (CLK_FREQ / 1000000))

void(*commutation_callback)();
volatile static uint8_t hall1_value = 0b000;
volatile static uint32_t hall1_period_uS = 0;


#ifdef EL_HALL1_ENABLED


uint8_t hall1_gpioRead()
{
    uint8_t state = 0;
    // Read each pin and shift into the correct bit position
    if (LL_GPIO_IsInputPinSet(EL_HALL1_A_PORT, EL_HALL1_A_PIN)) state |= (1 << 2); // MSB
    if (LL_GPIO_IsInputPinSet(EL_HALL1_B_PORT, EL_HALL1_B_PIN)) state |= (1 << 1);
    if (LL_GPIO_IsInputPinSet(EL_HALL1_C_PORT, EL_HALL1_C_PIN)) state |= (1 << 0); // LSB
    
    return state;
}

#define HALL1_GPIO_INIT(pin , port)do{\
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};\
    GPIO_InitStruct.Pin = pin;\
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;\
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;\
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;\
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;\
    LL_GPIO_Init(port, &GPIO_InitStruct);\
}while(0)

void el_hall1_init()
{   
    // Enable Timer 2 clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    
    // ===== TIME BASE CONFIGURATION =====
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler =  __LL_TIM_CALC_PSC(HAL_RCC_GetPCLK1Freq()*2, CLK_FREQ); // (1us resolution)
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFF; // Max 16-bit value (65.535ms)
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    // ===== HALL SENSOR CONFIGURATION =====
    LL_TIM_HALLSENSOR_InitTypeDef HALL_InitStruct = {0};
    HALL_InitStruct.IC1Polarity = LL_TIM_IC_POLARITY_BOTHEDGE; // Capture any Hall change
    HALL_InitStruct.IC1Prescaler = LL_TIM_ICPSC_DIV1;          // Capture every edge
    HALL_InitStruct.IC1Filter = 0x7;                           // 8 samples filtering
    HALL_InitStruct.CommutationDelay = 0;                      // Will set later
    LL_TIM_HALLSENSOR_Init(TIM2, &HALL_InitStruct);
    
    // ===== START TIMER =====
    LL_TIM_EnableCounter(TIM2);

    // ===== INTERRUPT CONFIGURATION =====
    // Enable capture/compare interrupt for Channel 1 (Hall changes)
    LL_TIM_EnableIT_CC1(TIM2);
    // Enable update interrupt for Channel 2 (commutation delay)
    LL_TIM_EnableIT_CC2(TIM2);
    
    // NVIC Configuration
    NVIC_SetPriority(TIM2_IRQn, 2); // Lower priority than PWM timer
    NVIC_EnableIRQ(TIM2_IRQn);

    HALL1_GPIO_INIT(EL_HALL1_A_PIN, EL_HALL1_A_PORT);
    HALL1_GPIO_INIT(EL_HALL1_B_PIN, EL_HALL1_B_PORT);
    HALL1_GPIO_INIT(EL_HALL1_C_PIN, EL_HALL1_C_PORT);

    //Arm Hall1 value
    hall1_value = hall1_gpioRead();
}

void el_hall1_setComDelay_uS(uint32_t delay_uS)
{
    uint32_t compare_value = US_TO_CLK(delay_uS);
    
    // Handle 16-bit timer overflow
    if (compare_value > 0xFFFF) {
        compare_value -= 0x10000;  // Wrap around
    }
    
    // Set the compare register
    LL_TIM_OC_SetCompareCH2(TIM2, compare_value);
}

// Set your commutation callback function
void el_hall1_setComCallback(void (*callback)(void))
{
    commutation_callback = callback;
}

float el_hall1_elec_speed(){
    return (((M_PI/3)*1000000)/hall1_period_uS);
}

uint8_t el_hall1_read(){
    return hall1_value;
}



#else

void el_comDelay_init()
{
    // Enable Timer 2 clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    
    // ===== TIME BASE CONFIGURATION =====
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler =  __LL_TIM_CALC_PSC(HAL_RCC_GetPCLK1Freq()*2, CLK_FREQ); // 84MHz/84 = 1MHz (1us resolution)
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xFFFF; // Max 16-bit value (65.535ms)
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    // Configure CH2 as output compare in one-pulse mode
    LL_TIM_OC_InitTypeDef OC_InitStruct = {0};
    OC_InitStruct.OCMode = LL_TIM_OCMODE_ACTIVE; // Channel goes active at compare match
    OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE; // Initially disabled
    OC_InitStruct.CompareValue = 0; // Will set later
    OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &OC_InitStruct);
    // Enable one pulse mode
    LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
    // Enable interrupt on CH2
    LL_TIM_EnableIT_CC2(TIM2);

    // NVIC
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Start counter disabled
    LL_TIM_DisableCounter(TIM2);
}

void el_comDelay_setComDelay_uS(uint32_t delay_uS)
{
    // Set compare value for CH2
    LL_TIM_OC_SetCompareCH2(TIM2, US_TO_CLK(delay_uS));

    // Reset counter
    LL_TIM_SetCounter(TIM2, 0);

    // Enable channel output
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);

    // Start timer
    LL_TIM_EnableCounter(TIM2);
}

void el_comDelay_setComCallback(void (*callback)(void))
{
    commutation_callback = callback;
}



#endif

// Timer 2 Interrupt Handler - THIS IS YOUR COMMUTATION EVENT!
void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM2) && LL_TIM_IsEnabledIT_CC1(TIM2) ) {
        LL_TIM_ClearFlag_CC1(TIM2);
        // HALL change happened , capture the period and update the hall_value using hall1_gpioRead();
        hall1_period_uS = CLK_TO_US(LL_TIM_IC_GetCaptureCH1(TIM2));
        
        #ifdef EL_HALL1_ENABLED
        hall1_value = hall1_gpioRead();
        #endif
    }
    if (LL_TIM_IsActiveFlag_CC2(TIM2) && LL_TIM_IsEnabledIT_CC2(TIM2)) {
        LL_TIM_ClearFlag_CC2(TIM2);
        // PERFORM COMMUTATION HERE
        if (commutation_callback != NULL) {
            commutation_callback();
        }
    }
}
