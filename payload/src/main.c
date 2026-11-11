#include "usb_device.h"
#include "usb_serial.h"
#include "tmp119.h"
#include "I2C_DMA.h"
#include "gpio.h"
#include "ICM.h"
#include "mpr.h"
#include "sdio.h"
#include "fatfs.h"
void SystemClock_Config();
void Error_Handler();
void gpio_init();

I2C_HandleTypeDef hi2c1;

struct ICM icm_sensor;
MPR_Data mprSensor;
struct tmp119 temp_sensor1;
struct tmp119 temp_sensor2;
float filteredAccel[3] = {-9999.0f, -9999.0f, -9999.0f};
float filteredGyro[3] = {-9999.0f, -9999.0f, -9999.0f};
float filteredTemp[2] = {-9999.0f, -9999.0f};
enum STATE
{
	IDLE,
	ARMED,
	ASCENT,
	DESCENT,
	RECOVERY,
	ERRORSTATE
};
const char *stateNames[] = {
	"IDLE",
	"ARMED",
	"ASCENT",
	"DESCENT",
	"RECOVERY",
	"ERRORSTATE"};
enum STATE activeState;
bool stateChangeComplete = false;
float prevPressure = -9999.0f;
uint32_t ascentStart = 0;
uint32_t descentStart = 0;
int descentCounter = 0;
int ascentCounter = 0;
double minPressureSeen = 999.0;
double maxPressureSeen = 0.0;
void exponentialFilter(float input, float *previous, double alpha);
double alpha = 0;
FATFS FatFs;
FIL Fil;
FRESULT FR_Status;
FATFS *FS_Ptr;
char filename[16];
FILINFO fno;
UINT RWC, WWC; // Read/Write Word Counter
DWORD FreeClusters;
uint32_t TotalSize, FreeSpace;
char RW_Buffer[200];
BYTE work[512] __attribute__((aligned(4)));
int countSinceLastSync = 0;
int main()
{
	activeState = IDLE;

	HAL_Init();

	SystemClock_Config();
	LED_Init();
	MX_USB_DEVICE_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	
	I2C_Init(&hi2c1);
	gpio_init();
	HAL_Delay(2000);
	ICM_init(&icm_sensor);
	TMP119_setAddr(&temp_sensor1, false);
	TMP119_setAddr(&temp_sensor2, true);
	FR_Status = f_mount(&FatFs, SDPath, 0);
    if (FR_Status != FR_OK)
    {
      SerialWrite("Error! While Mounting SD Card 0 , Error Code: (%i)\r\n", FR_Status);
    }
	
	FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      SerialWrite("Error! While Mounting SD Card 1, Error Code: (%i)\r\n", FR_Status);
    }
	for (int i = 0; i < 1000; i++) { snprintf(filename, sizeof(filename), "log%03d.txt", i); if (f_stat(filename, &fno) == FR_NO_FILE) break; }
	FR_Status = f_open(&Fil, filename, FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	HAL_Delay(1000);
	while (1)
	{

		switch (activeState)
		{
		case IDLE:
			if (!stateChangeComplete)
			{	
				f_puts("SYSTEM ON\n", &Fil);
				f_puts("STATE: IDLE\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				TMP119_SetLowPower(&temp_sensor1, true);
				TMP119_SetLowPower(&temp_sensor2, true);
				ICM_setLowPower(true);
				// mpr is low power by default when not reading
				stateChangeComplete = true;
			}
			SerialWrite("time %u \n",HAL_GetTick());
			// broken by radio command to "ARM" - switch to case ARMED
			// all but radio off
			if (HAL_GetTick()>10000) //TODO: PUT RADIO TRIGGER HERE
			 {
			 stateChangeComplete = false;
			 activeState = ARMED;
			}
			break;
		case ARMED:
			if (!stateChangeComplete)
			{
				f_puts("STATE: ARMED\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				alpha = .9;
				ICM_setLowPower(false);
				stateChangeComplete = true;
			}
			if (!icm_sensor.aBusy && !mprSensor.busy)
			{
				HAL_StatusTypeDef ret = ICM_StartAccel(&icm_sensor);
				if (ret != HAL_OK)
				{
					activeState = ERRORSTATE;
				}
				while (icm_sensor.aBusy)
					;

				MPR_errors read_result = MPR_Read(&mprSensor);
				while (mprSensor.busy)
					;
				if (read_result != MPR_OK)
				{
					activeState = ERRORSTATE;
				}
				if (icm_sensor.new_accel_data)
				{
					exponentialFilter(icm_sensor.accelX, (&filteredAccel[0]), alpha);
					exponentialFilter(icm_sensor.accelY, (&filteredAccel[1]), alpha);
					exponentialFilter(icm_sensor.accelZ, (&filteredAccel[2]), alpha);
					exponentialFilter(mprSensor.pressure, (&prevPressure), alpha);
					SerialWrite("Accel, %f, Pressure, %.2f psi | asce:%d \n", icm_sensor.accelY, prevPressure, ascentCounter);
					int n = snprintf(RW_Buffer, sizeof(RW_Buffer), "Accel, %f, Pressure, %.2f psi | AscentTrigger:%d \n", icm_sensor.accelY, prevPressure, ascentCounter);
				if (n > 0)
				{
					UINT bw;
					f_write(&Fil, RW_Buffer, (n < (int)sizeof(RW_Buffer)) ? n : sizeof(RW_Buffer) - 1, &bw);
				}
					icm_sensor.new_accel_data = false;
				}
			}

			// broken by barometer detecting large spike - "launch", switch to case ASCENT
			// broken by sensor reading error  - something not recieving, radio com. failing, switch to case ERROR
			// imu on, accellerometer on, TMP off, pressure low reading?
			if (prevPressure > 14 && filteredAccel[2] < .5)
			{
				if (prevPressure > maxPressureSeen)
				{
					maxPressureSeen = prevPressure;
					ascentCounter = 0;
				}
				else if (prevPressure < maxPressureSeen - 0.005)
				{
					ascentCounter++;
				}
				else if (ascentCounter > 0)
				{
					ascentCounter--;
				}

				if (ascentCounter > 75)
				{
					stateChangeComplete = false;
					activeState = ASCENT;
				}
			}
			countSinceLastSync++;
			if (countSinceLastSync > 250)
			{
				f_sync(&Fil);
				countSinceLastSync = 0;
			}
			break;
		case ASCENT:
			if (!stateChangeComplete)
			{
				ascentStart=HAL_GetTick();
				f_puts("STATE: ASCENT\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				alpha = .9;
				TMP119_SetLowPower(&temp_sensor1, false);
				TMP119_SetLowPower(&temp_sensor2, false);
				ICM_setLowPower(false);
				stateChangeComplete = true;
			}
			// make sure last loop is "done" before starting the next one
			if (!icm_sensor.aBusy &&
				!icm_sensor.gBusy &&
				!temp_sensor1.busy && !temp_sensor2.busy && !mprSensor.busy)
			{
				// here is where you ensure that all the sensors are ready to be read from, and then you can start the next read cycle
				ICM_StartAccel(&icm_sensor);
				while (icm_sensor.aBusy)
					;
				ICM_StartGyro(&icm_sensor);
				while (icm_sensor.gBusy)
					;
				TMP119_StartRead(&temp_sensor1);
				while (temp_sensor1.busy)
					;
				TMP119_StartRead(&temp_sensor2);
				while (temp_sensor2.busy)
					;
				MPR_Read(&mprSensor);
				while (mprSensor.busy)
					;
			}
			if (icm_sensor.new_accel_data &&
				icm_sensor.new_gyro_data &&
				temp_sensor1.new_data && temp_sensor2.new_data)
			{
				uint32_t elapsed = HAL_GetTick() - ascentStart;
				exponentialFilter(icm_sensor.accelX, (&filteredAccel[0]), alpha);
				exponentialFilter(icm_sensor.accelY, (&filteredAccel[1]), alpha);
				exponentialFilter(icm_sensor.accelZ, (&filteredAccel[2]), alpha);
				exponentialFilter(icm_sensor.gyroX, (&filteredGyro[0]), alpha);
				exponentialFilter(icm_sensor.gyroY, (&filteredGyro[1]), alpha);
				exponentialFilter(icm_sensor.gyroZ, (&filteredGyro[2]), alpha);
				exponentialFilter(temp_sensor1.temp, (&filteredTemp[0]), alpha);
				exponentialFilter(temp_sensor2.temp, (&filteredTemp[1]), alpha);
				exponentialFilter(mprSensor.pressure, (&prevPressure), alpha);
				// this is where all the stuff is properly updated and should be referenced/called from
				SerialWrite("t:%u, | ax:%.2f,ay:%.2f,az:%.2f,| gx:%.2f,gy:%.2f,gz:%.2f, | t1:%.2f,t2:%.2f, | p:%.4f, | desc:%d \n",
							(unsigned long)elapsed,
							filteredAccel[0], filteredAccel[1], filteredAccel[2],
							filteredGyro[0], filteredGyro[1], filteredGyro[2],
							filteredTemp[0], filteredTemp[1], prevPressure, descentCounter);
				int n = snprintf(RW_Buffer, sizeof(RW_Buffer), "t:%lu, | ax:%.2f,ay:%.2f,az:%.2f,| gx:%.2f,gy:%.2f,gz:%.2f, | t1:%.2f,t2:%.2f, | p:%.4f, | desc:%d \r\n",
								 (unsigned long)elapsed,
								 filteredAccel[0], filteredAccel[1], filteredAccel[2],
								 filteredGyro[0], filteredGyro[1], filteredGyro[2],
								 filteredTemp[0], filteredTemp[1], prevPressure, descentCounter);
				if (n > 0)
				{
					UINT bw;
					f_write(&Fil, RW_Buffer, (n < (int)sizeof(RW_Buffer)) ? n : sizeof(RW_Buffer) - 1, &bw);
				}
				icm_sensor.new_accel_data = false;
				icm_sensor.new_gyro_data = false;
				temp_sensor1.new_data = false;
				temp_sensor2.new_data = false;
			}
			countSinceLastSync++;
			if (countSinceLastSync > 250)
			{
				f_sync(&Fil);
				countSinceLastSync = 0;
			}
			
			// launch timer begins?
			// broken by baro differntial being consistent - means  apogee hit, switch to case DESCENT
			// all sensors on reading full
			if (prevPressure < 14.6)
			{ // update for realistic apogee height
				if (prevPressure < minPressureSeen)
				{
					minPressureSeen = prevPressure;
					descentCounter = 0;
				}
				else if (prevPressure > minPressureSeen + 0.003)
				{ 
					descentCounter++;
				}
				else if (descentCounter > 0)
				{
					descentCounter--;
				}

				if (descentCounter > 75)
				{
					stateChangeComplete = false;
					activeState = DESCENT;
				}
			}
			break;
		case DESCENT:
			if (!stateChangeComplete)
			{
				descentStart = HAL_GetTick();
				f_puts("STATE: DESCENT\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				stateChangeComplete = true;
			}
			// broken by velocity going to ~0, baro reading going to ~ starting alt. Switch to case RECOVERY
			 if (HAL_GetTick()-descentStart>10000)
			{
			 stateChangeComplete = false;
			 activeState = RECOVERY;
			}
			SerialWrite("descent\n");
			break;
		case RECOVERY: 
			if (!stateChangeComplete)
			{
				f_puts("STATE: RECOVERY\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				stateChangeComplete = true;
			}
			SerialWrite("recover\n");
			countSinceLastSync++;
			if (countSinceLastSync > 250)
			{
				f_sync(&Fil);
				countSinceLastSync = 0;
			}
			// final state
			// all but radio off/save
			//f_close(&Fil);
			break;
		case ERRORSTATE:
			if (!stateChangeComplete)
			{
				f_puts("STATE: ERROR\n", &Fil);
				f_sync(&Fil);
				countSinceLastSync=0;
				stateChangeComplete = true;
			}
			SerialWrite("error\n");
			countSinceLastSync++;
			if (countSinceLastSync > 250)
			{
				f_sync(&Fil);
				countSinceLastSync = 0;
			}

			break;
		}
		// SerialWrite("Current State: %s\n", stateNames[activeState]);
		HAL_Delay(10);
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
	else if (temp_sensor1.busy)
	{
		TMP119_GetTemperature(&temp_sensor1);
	}
	else if (temp_sensor2.busy)
	{
		TMP119_GetTemperature(&temp_sensor2);
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
void exponentialFilter(float input, float(*previous), double alpha)
{
	if (*previous == -9999.0f)
	{
		*previous = input;
		return;
	}
	float filtered = (*previous) + (1 - alpha) * (input - (*previous));
	*previous = filtered;
	return;
}