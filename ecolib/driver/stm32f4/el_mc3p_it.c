
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "el_mc3p.h"
#include "eld_conf.h"

#ifdef EL_MC3P_ENABLED

TIM_HandleTypeDef        htim11;

/**
  * @brief  This function configures the TIM11 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  HAL_StatusTypeDef     status;

  /* Enable TIM11 clock */
  __HAL_RCC_TIM11_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM11 clock */
      uwTimclock = HAL_RCC_GetPCLK2Freq();

  /* Compute the prescaler value to have TIM11 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM11 */
  htim11.Instance = TIM11;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM11CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim11.Init.Period = (1000000U / (EL_XMC3P_TICKFREQ)) - 1U;
  htim11.Init.Prescaler = uwPrescalerValue;
  htim11.Init.ClockDivision = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim11);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim11);
    if (status == HAL_OK)
    {
    /* Enable the TIM11 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
        HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TickPriority, 2U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM11 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM11 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM11 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM11 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
}

//==============================================================================

__attribute__((weak)) void el_xmc3p_tickerCallback(void){}
__attribute__((weak)) void el_mc3p_sync_postScanCallback(void){}

static el_mc3p_handle_t* s_mc3p = NULL;
void mc3p_irq_bind(el_mc3p_handle_t* h)
{
    s_mc3p = h;
}
__attribute__((weak)) void INTERNAL_mc3p_ADC_JEOS_IRQ(el_mc3p_handle_t *h){}

void ADC_IRQHandler(void)
{
if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
    LL_ADC_ClearFlag_JEOS(ADC1);
    INTERNAL_mc3p_ADC_JEOS_IRQ(s_mc3p);
    el_mc3p_sync_postScanCallback();
}
}
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
        LL_TIM_ClearFlag_UPDATE(TIM1);
    }
}

#define pTASK(task_name, ticker, task, divisor)do{\
    static uint16_t counter_##task_name = 0;\
    if(++counter_##task_name >= divisor){\
        counter_##task_name = 0;\
        task;\
    }\
}\
while(0);


void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    //=============TIMER 11 UPDATE INTERRUPT==============// USED for HAL_Ticker and MC_Scheduler
    if (LL_TIM_IsActiveFlag_UPDATE(TIM11)){
        LL_TIM_ClearFlag_UPDATE(TIM11);
        //Here We share Timer11 for HAL_ticking and motor Control Scheduling
        static uint8_t trg_cnt = 0;
        el_xmc3p_tickerCallback();
        pTASK(HAL_TICKER, trg_cnt, HAL_IncTick(), EL_XMC3P_TICKFREQ/1000);
        trg_cnt++;
    }
}

#endif