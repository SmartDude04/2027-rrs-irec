#include "usb_device.h"
#include "usb_serial.h"
#include "tmp119.h"
#include "I2C_DMA.h"
#include "gpio.h"
#include "ICM.h"
#include "mpr.h"


void SystemClock_Config();
void Error_Handler();
void gpio_init();


I2C_HandleTypeDef hi2c1;



struct ICM icm_sensor;
struct MPR_DATA mpr_sensor;

int main() {
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    MX_USB_DEVICE_Init();
    I2C_Init(&hi2c1);
	
	gpio_init();
	SerialWrite("Starting ICM-20948 test...\n");
	ICM_init(&icm_sensor);
	
    while (1) {
        /*if (!icm_sensor.busy) {
            ICM_StartAccel(&icm_sensor);
        }
		
        if (icm_sensor.new_data) {
			
			SerialWrite("X: %f  Y: %f  Z: %f\n",icm_sensor.accelX, icm_sensor.accelY, icm_sensor.accelZ);
            icm_sensor.new_data = false;
        }
		*/
		if (!mpr_sensor.busy) {
			MPR_Read(&mpr_sensor);
		}
		printf("Pressure: %f psi\n", mpr_sensor.pressure);
        HAL_Delay(10);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if (hi2c == &hi2c1) {
        ICM_GetAccel(&icm_sensor);
    }
}

void gpio_init() {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
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