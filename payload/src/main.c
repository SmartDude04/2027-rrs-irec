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
MPR_Data mprSensor;
struct tmp119 temp_sensor;

int main()
{
	HAL_Init();
	SystemClock_Config();
	LED_Init();
	MX_USB_DEVICE_Init();
	I2C_Init(&hi2c1);

	gpio_init();

	SerialWrite("Starting ICM-20948 test...\n");
	HAL_Delay(2000);
	ICM_init(&icm_sensor);

	while (1)
	{
		// make sure last loop is "done" before starting the next one
		if (!icm_sensor.aBusy &&
			!icm_sensor.gBusy &&
			!temp_sensor.busy)
		{
			// here is where you ensure that all the sensors are ready to be read from, and then you can start the next read cycle
			ICM_StartAccel(&icm_sensor);
			while (icm_sensor.aBusy)
				;
			ICM_StartGyro(&icm_sensor);
			while (icm_sensor.gBusy)
				;
			TMP119_StartRead(&temp_sensor);
			while (temp_sensor.busy)
				;
		}
		if (icm_sensor.new_accel_data &&
			icm_sensor.new_gyro_data &&
			temp_sensor.new_data)
		{
			// this is where all the stuff is properly updated and should be referenced/called from
			SerialWrite("A: %f, %f, %f | G: %f, %f, %f | T: %f\n",
						icm_sensor.accelX, icm_sensor.accelY, icm_sensor.accelZ,
						icm_sensor.gyroX, icm_sensor.gyroY, icm_sensor.gyroZ,
						temp_sensor.temp);

			icm_sensor.new_accel_data = false;
			icm_sensor.new_gyro_data = false;
			temp_sensor.new_data = false;
		}

		HAL_Delay(100);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if (hi2c != &hi2c1)
	{
		return;
	}

	if (icm_sensor.aBusy)
	{
		ICM_GetAccel(&icm_sensor);
	}
	else if (icm_sensor.gBusy)
	{
		ICM_GetGyro(&icm_sensor, false);
	}
	else if (temp_sensor.busy)
	{
		TMP119_GetTemperature(&temp_sensor);
	}
}

void gpio_init()
{
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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