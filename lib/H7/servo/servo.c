#include "servo.h"

#include "stm32h7xx_hal.h"

static void PWM_GPIO_Init_PA6(void);
static void PWM_TIM3_Init(void);

static TIM_HandleTypeDef tim3_handle;

void Servo_Init_TIM3_PA6(void)
{
    PWM_GPIO_Init_PA6();
    PWM_TIM3_Init();
    
    if (HAL_TIM_PWM_Start(&tim3_handle, TIM_CHANNEL_1) != HAL_OK) {
        while (1) { }
    }
}

void Servo_Move_Degrees(uint8_t degrees)
{
    degrees = degrees > 180U ? 180U : degrees;
    const uint32_t period_us = 500U + (degrees * 2000U) / 180U;
    __HAL_TIM_SET_COMPARE(&tim3_handle, TIM_CHANNEL_1, period_us);
}

void PWM_GPIO_Init_PA6(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // Enable GPIOA Clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure PA6 to use TIM3_CH1
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PWM_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // Configure the static global variable for this timer
    tim3_handle.Instance = TIM3;
    tim3_handle.Init.Prescaler = 63; // Sets the timer to have a realized frequency of 1 MHz
    tim3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim3_handle.Init.Period = 19999; // Period of 20 ms (I believe this is correct... couldn't find it on the datasheet)
    tim3_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim3_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    HAL_TIM_PWM_Init(&tim3_handle);
    
    // Configure the PWM chanel
    TIM_OC_InitTypeDef output_compare_config = {0};
    output_compare_config.OCMode = TIM_OCMODE_PWM1;
    output_compare_config.Pulse = 499; // Initial pulse width of 500 us
    output_compare_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    output_compare_config.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&tim3_handle, &output_compare_config, TIM_CHANNEL_1);
}