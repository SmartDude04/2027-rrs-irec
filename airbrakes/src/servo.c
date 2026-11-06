#include "servo.h"
#include "stm32h7xx_hal.h"

TIM_HandleTypeDef htim16;
/*

Must be called to declare the PWM pin
*/


void FINNIAN_TIM16_Init(void){

    __HAL_RCC_GPIOF_CLK_ENABLE(); 
    __HAL_RCC_TIM16_CLK_ENABLE(); //source clock for TIM16 enable

    /* PF6 = TIM16_CH1, AF1 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOF, &gpio);

    //stop the timer + disable compare values while configuring
    TIM16->CR1 = 0;
    TIM16->CCER &= ~(1U << 0);  // CC1E = 0 

    
    

    //1. Configure the output pin:
    //a) Select the output mode by writing CCS bits in CCMRx register. 
    TIM16->CCMR1 &= ~11; //configures channel 1 as output

    //b) Select the polarity by writing the CCxP bit in CCER register.
    TIM16->CCER &= ~((1U << 1) | (1U << 3)); //configures channel 1 as active high

    //2. Select the PWM mode (PWM1 or PWM2) by writing OCxM bits in CCMRx register.
    TIM16->CCMR1 |= (1<<6) | (1<<5) & ~(1<<4); //configures channel 1 as PWM1, bits 16 = 0, 6:4 = 110

    //3. Program the period and the duty cycle respectively in ARR and CCRx registers.
    TIM16->PSC  = 63; // 64MHz / (63+1) gives a 1Mhz timer clock, 1 tick per microsecond
    TIM16->ARR  = 19999U;                              // 20 ms, 1 tick per microsecond, 50hz
    TIM16->CCR1 = 1500U;                               // 1,500 microseconds

    //4. Set the preload bit in CCMRx register and the ARPE bit in the CR1 register.
    TIM16->CCMR1 |= (1<<3); //enable preload for channel 1
    TIM16->CR1 |= (1<<7); //enable auto reload preload

    //5. Select the counting mode:
    //a) PWM edge-aligned mode: the counter must be configured up-counting or
    //   down-counting
    //b) PWM center aligned mode: the counter mode must be center aligned counting
    //   mode (CMS bits different from '00').

    //should be taken care of?

    //6. Enable the capture compare.

    TIM16->CCER |= 1; //enable channel 1 output

    //7. Enable the counter.

    TIM16->CR1 |= 1; //enable counter


}




/*

lets you set the pulse width in microseconds. 500 = 0 degrees, 2500 = 180 degrees. every 11.1 us is a degree

*/
void Servo_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse_us);
}

/* You need to call this once after MX_TIM16_Init() in our main function */
void Servo_Start()
{
    if (HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}


/*
This function is executed when an error in config happens
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}