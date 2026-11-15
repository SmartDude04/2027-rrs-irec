#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "usb_device.h"
#include "usb_serial.h"
#include "tmp119.h"
#include "I2C_DMA.h"
#include "ICM.h"
#include "mpr.h"
#include "sdio.h"
#include "fatfs.h"
#include "lora_sx1276.h"
#include "radio.h"
#include "SPI.h"
#include "message_types.h"
#include "protocol.h"
void SystemClock_Config();
void Error_Handler();
void gpio_init();
void exponentialFilter(float input, float *previous, double alpha);
bool sensorReadWrite(void);
float getAltFromLaunchM(float currentAltPSI);
float mtoft(float meters);
float psiToM(float psi);
float calibrateOffsetPSI(float knownAltM, int CAL_NUM_READS, double alpha);
I2C_HandleTypeDef hi2c2;

lora_sx1276 lora = {
	.frequency = 433000000,
	.nss_pin = GPIO_PIN_2,
	.nss_port = GPIOA,
	.rst_pin = GPIO_PIN_1,
	.rst_port = GPIOA,
	.spi = &hspi1,
	.bw = LORA_BANDWIDTH_125_KHZ,
	.sf = LORA_DEFAULT_SF,
	.pa_mode = LORA_PA_OUTPUT_PA_BOOST,
	.rx_base_addr = LORA_DEFAULT_RX_ADDR,
	.tx_base_addr = LORA_DEFAULT_TX_ADDR,
	.tx_power = LORA_DEFAULT_TX_POWER,
	.rx_frequency_offset = 0,
	.spi_timeout = LORA_DEFAULT_SPI_TIMEOUT};
struct radio_link link;


struct ICM icm_sensor;
MPR_Data mprSensor;
struct tmp119 temp_sensor1;
struct tmp119 temp_sensor2;
float prevPressure = -9999.0f;
double minPressureSeen = 999.0;
double maxPressureSeen = 0.0;
double alpha = .35;
const uint32_t SENSOR_TIMEOUT_MS = 20;
const uint32_t RECOVERY_INTERVAL_MS = 10000;
uint32_t SENSOR_ERROR[5] = {0}; // accel, gyro, temp1, temp2, mpr
double LAUNCH_ALTITUDE_M = 67.0;
double APOGEE_SEARCH_ALTITUDE_M = 800.0;
float offsetPSI = 0;
float filteredAccel[3] = {-9999.0f, -9999.0f, -9999.0f};
float filteredGyro[3] = {-9999.0f, -9999.0f, -9999.0f};
float filteredTemp[2] = {-9999.0f, -9999.0f};

enum STATE
{
	IDLE,
	ARMED,
	ASCENT,
	DESCENT,
	RECOVERY
};
const char *stateNames[] = {
	"IDLE",
	"ARMED",
	"ASCENT",
	"DESCENT",
	"RECOVERY"};
enum STATE activeState;
bool stateChangeComplete = false;

uint32_t ascentStart = 0;
uint32_t descentStart = 0;
uint32_t recoveryRead = 0;
uint32_t timeLastSync = 0;
int descentCounter = 0;
int ascentCounter = 0;
int recoverycount = 0;

FATFS FatFs;
FIL Fil;
FRESULT FR_Status;
FATFS *FS_Ptr;
char filename[16];
FILINFO fno;
char radioPayload[128];
uint8_t radio_len = 0;
bool ARM_FLAG = false;

int main()
{
	activeState = IDLE;
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	I2C_Init(&hi2c2);
	SPI_Init();
	MX_USB_DEVICE_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	radio_link_init(&lora, &link, NODE_PAYLOAD);
	HAL_Delay(1000);
	ICM_init(&icm_sensor);
	TMP119_setAddr(&temp_sensor1, false);
	TMP119_setAddr(&temp_sensor2, true);
	FR_Status = f_mount(&FatFs, SDPath, 0);

	if (FR_Status != FR_OK)
	{
		while (1)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(400);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(1000);
		}
	}
	FR_Status = f_mount(&FatFs, SDPath, 1);
	if (FR_Status != FR_OK)
	{
		while (1)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(400);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(1000);
		}
	}
	for (int i = 0; i < 1000; i++)
	{
		snprintf(filename, sizeof(filename), "log%03d.txt", i);
		if (f_stat(filename, &fno) == FR_NO_FILE)
			break;
	}
	FR_Status = f_open(&Fil, filename, FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	HAL_Delay(1000);
	while (1)
	{

		switch (activeState)
		{
		case IDLE:
			if (!stateChangeComplete)
			{

				f_puts("ms,state,alt,ax,ay,az,gx,gy,gz,t1,t2,p,rax,ray,raz,rgx,rgy,rgz,rt1,rt2,rp,err_acc,err_gyr,err_t1,err_t2,err_mpr,err_sd,rssi,snr\n", &Fil);
				f_sync(&Fil);
				TMP119_SetLowPower(&temp_sensor1, true);
				TMP119_SetLowPower(&temp_sensor2, true);
				ICM_setLowPower(true);
				// mpr is low power by default when not reading
				stateChangeComplete = true;
			}
			// broken by radio command to "ARM" - switch to case ARMED
			// all but radio off
			if (ARM_FLAG)
			{
				stateChangeComplete = false;
				activeState = ARMED;
			}
			int n = snprintf(radioPayload, sizeof(radioPayload), "%lu,%s",
							 (unsigned long)HAL_GetTick(), stateNames[activeState]);
			if (n < 0)
				n = 0;
			if (n >= (int)sizeof(radioPayload))
				n = sizeof(radioPayload) - 1;
			radio_len = (uint8_t)n;
			break;
		case ARMED:
			if (!stateChangeComplete)
			{
				offsetPSI = calibrateOffsetPSI(LAUNCH_ALTITUDE_M, 100, alpha);
				f_sync(&Fil);
				// alpha = .75;
				TMP119_SetLowPower(&temp_sensor1, false);
				TMP119_SetLowPower(&temp_sensor2, false);
				ICM_setLowPower(false);
				stateChangeComplete = true;
			}
			sensorReadWrite();

			// broken by barometer detecting large spike - "launch", switch to case ASCENT
			// broken by sensor reading error  - something not recieving, radio com. failing, switch to case ERROR
			// imu on, accellerometer on, TMP off, pressure low reading?
			if (psiToM(prevPressure) > LAUNCH_ALTITUDE_M && fabsf(filteredAccel[2]) > 0.5)
			{
				if (psiToM(prevPressure) < psiToM(maxPressureSeen))
				{
					maxPressureSeen = prevPressure;
					ascentCounter = 0;
				}
				else if (psiToM(prevPressure) > psiToM(maxPressureSeen) + 0.005)
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
			break;
		case ASCENT:
			if (!stateChangeComplete)
			{
				ascentStart = HAL_GetTick();
				f_sync(&Fil);
				// alpha = .75;
				TMP119_SetLowPower(&temp_sensor1, false);
				TMP119_SetLowPower(&temp_sensor2, false);
				ICM_setLowPower(false);
				stateChangeComplete = true;
			}
			sensorReadWrite();

			// launch timer begins?
			// broken by baro differntial being consistent - means  apogee hit, switch to case DESCENT
			// all sensors on reading full
			if (psiToM(prevPressure) > APOGEE_SEARCH_ALTITUDE_M)
			{ // update for realistic apogee height
				if (psiToM(prevPressure) > psiToM(minPressureSeen))
				{
					minPressureSeen = prevPressure;
					descentCounter = 0;
				}
				else if (psiToM(prevPressure) < psiToM(minPressureSeen) - .005)
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
				f_sync(&Fil);
				// alpha = .75;
				stateChangeComplete = true;
			}
			sensorReadWrite();
			// broken by velocity going to ~0, baro reading going to ~ starting alt. Switch to case RECOVERY
			if (psiToM(prevPressure) < LAUNCH_ALTITUDE_M + 5 && fabsf(filteredAccel[2]) < 0.5)
			{
				stateChangeComplete = false;
				activeState = RECOVERY;
			}

			break;
		case RECOVERY:
			if (!stateChangeComplete)
			{
				f_sync(&Fil);

				stateChangeComplete = true;
			}
			if ((HAL_GetTick() - recoveryRead) >= RECOVERY_INTERVAL_MS)
			{
				recoveryRead = HAL_GetTick();
				sensorReadWrite();
			}
			break;
		}
		radio_rx_process(&link);
		
		if (radio_flag == MSG_TELEMETRY)
		{
			radio_link_send(&link, NODE_GROUND_STATION, MSG_TELEMETRY_ACK, (uint8_t *)radioPayload, strlen(radioPayload));
			radio_flag = 0;
		}
		if (radio_flag == MSG_COMMAND_ARM &&activeState == IDLE)
		{

			ARM_FLAG = true;
			radio_flag = 0;
		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(10);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	
	if (hi2c != &hi2c2)
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

void exponentialFilter(float input, float *previous, double alpha)
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
bool sensorReadWrite(void)
{
	static uint32_t sdErrorCount = 0;
	bool readError[5] = {false};
	uint32_t readstart;
	char line[256];
	int pos = snprintf(line, sizeof(line), "%lu,%s", (unsigned long)HAL_GetTick(), stateNames[activeState]);
	icm_sensor.new_accel_data = false;
	icm_sensor.new_gyro_data = false;
	temp_sensor1.new_data = false;
	temp_sensor2.new_data = false;

	/* ---------- accel ---------- */
	readstart = HAL_GetTick();
	ICM_StartAccel(&icm_sensor);
	while (icm_sensor.aBusy)
	{
		if ((HAL_GetTick() - readstart) > SENSOR_TIMEOUT_MS)
		{
			if (hi2c2.hdmarx != NULL)
				HAL_DMA_Abort(hi2c2.hdmarx);
			HAL_I2C_DeInit(&hi2c2);
			I2C_Init(&hi2c2);
			icm_sensor.aBusy = false;
			icm_sensor.new_accel_data = false;
			break;
		}
	}

	/* ---------- gyro ---------- */
	readstart = HAL_GetTick();
	ICM_StartGyro(&icm_sensor);
	while (icm_sensor.gBusy)
	{
		if ((HAL_GetTick() - readstart) > SENSOR_TIMEOUT_MS)
		{
			if (hi2c2.hdmarx != NULL)
				HAL_DMA_Abort(hi2c2.hdmarx);
			HAL_I2C_DeInit(&hi2c2);
			I2C_Init(&hi2c2);
			icm_sensor.gBusy = false;
			icm_sensor.new_gyro_data = false;
			break;
		}
	}

	/* ---------- temp 1 ---------- */
	readstart = HAL_GetTick();
	TMP119_StartRead(&temp_sensor1);
	while (temp_sensor1.busy)
	{
		if ((HAL_GetTick() - readstart) > SENSOR_TIMEOUT_MS)
		{
			if (hi2c2.hdmarx != NULL)
				HAL_DMA_Abort(hi2c2.hdmarx);
			HAL_I2C_DeInit(&hi2c2);
			I2C_Init(&hi2c2);
			temp_sensor1.busy = false;
			temp_sensor1.new_data = false;
			break;
		}
	}

	/* ---------- temp 2 ---------- */
	readstart = HAL_GetTick();
	TMP119_StartRead(&temp_sensor2);
	while (temp_sensor2.busy)
	{
		if ((HAL_GetTick() - readstart) > SENSOR_TIMEOUT_MS)
		{
			if (hi2c2.hdmarx != NULL)
				HAL_DMA_Abort(hi2c2.hdmarx);
			HAL_I2C_DeInit(&hi2c2);
			I2C_Init(&hi2c2);
			temp_sensor2.busy = false;
			temp_sensor2.new_data = false;
			break;
		}
	}

	MPR_errors mprResult = MPR_Read(&mprSensor);
	while (mprSensor.busy)
		;

	readError[0] = !icm_sensor.new_accel_data;
	readError[1] = !icm_sensor.new_gyro_data;
	readError[2] = !temp_sensor1.new_data;
	readError[3] = !temp_sensor2.new_data;
	readError[4] = (mprResult != MPR_OK);

	for (int i = 0; i < 5; i++)
		if (readError[i])
			SENSOR_ERROR[i]++;

	if (!readError[0])
	{
		exponentialFilter(icm_sensor.accelX, &filteredAccel[0], alpha);
		exponentialFilter(icm_sensor.accelY, &filteredAccel[1], alpha);
		exponentialFilter(icm_sensor.accelZ, &filteredAccel[2], alpha);
	}
	if (!readError[1])
	{
		exponentialFilter(icm_sensor.gyroX, &filteredGyro[0], alpha);
		exponentialFilter(icm_sensor.gyroY, &filteredGyro[1], alpha);
		exponentialFilter(icm_sensor.gyroZ, &filteredGyro[2], alpha);
	}
	if (!readError[2])
		exponentialFilter(temp_sensor1.temp, &filteredTemp[0], alpha);
	if (!readError[3])
		exponentialFilter(temp_sensor2.temp, &filteredTemp[1], alpha);
	if (!readError[4])
		exponentialFilter(mprSensor.pressure, &prevPressure, alpha);

	float vals[9] = {
		filteredAccel[0], filteredAccel[1], filteredAccel[2],
		filteredGyro[0], filteredGyro[1], filteredGyro[2],
		filteredTemp[0], filteredTemp[1], prevPressure};
	float rawVals[9] = {
		(float)icm_sensor.rawX, (float)icm_sensor.rawY, (float)icm_sensor.rawZ,
		(float)icm_sensor.rawGyroX, (float)icm_sensor.rawGyroY, (float)icm_sensor.rawGyroZ,
		(float)temp_sensor1.raw, (float)temp_sensor2.raw, (float)mprSensor.pressure};
	bool good[9] = {
		!readError[0], !readError[0], !readError[0],
		!readError[1], !readError[1], !readError[1],
		!readError[2], !readError[3], !readError[4]};
	{
		float altM = psiToM(prevPressure);
		if (pos < (int)sizeof(line) - 1)
		{
			int altN = (!readError[4] && !isnan(altM))
						   ? snprintf(line + pos, sizeof(line) - pos, ",%.2f", altM)
						   : snprintf(line + pos, sizeof(line) - pos, ",-9999");
			if (altN > 0)
				pos += altN;
		}
	}
	for (int i = 0; i < 9 && pos < (int)sizeof(line) - 1; i++)
	{
		int n;
		if (good[i])
			n = snprintf(line + pos, sizeof(line) - pos, ",%.*f", (i == 8) ? 4 : 2, vals[i]);
		else
			n = snprintf(line + pos, sizeof(line) - pos, ",-9999");
		if (n > 0)
			pos += n;
	}
	// raw columns: same validity flags, same column count whether good or bad
	for (int i = 0; i < 9 && pos < (int)sizeof(line) - 1; i++)
	{
		int n;
		if (good[i])
			n = snprintf(line + pos, sizeof(line) - pos, ",%.*f", (i == 8) ? 4 : 0, rawVals[i]);
		else
			n = snprintf(line + pos, sizeof(line) - pos, ",-9999");
		if (n > 0)
			pos += n;
	}
	if (pos < (int)sizeof(line) - 1)
	{
		int n = snprintf(line + pos, sizeof(line) - pos, ",%lu,%lu,%lu,%lu,%lu,%lu\r\n",
						 (unsigned long)SENSOR_ERROR[0], (unsigned long)SENSOR_ERROR[1],
						 (unsigned long)SENSOR_ERROR[2], (unsigned long)SENSOR_ERROR[3],
						 (unsigned long)SENSOR_ERROR[4], (unsigned long)sdErrorCount);
		if (n > 0)
			pos += n;
	}
	if (pos < (int)sizeof(line) - 1)
{
	int n = snprintf(line + pos, sizeof(line) - pos, ",%4.2d", &link.stats.last_rssi_dbm);
	if (n > 0)
		pos += n;
}
if (pos < (int)sizeof(line) - 1)
{
	int n = snprintf(line + pos, sizeof(line) - pos, ",%4.2d", &link.stats.last_snr_db);
	if (n > 0)
		pos += n;
}
	if (pos > (int)sizeof(line) - 1)
		pos = sizeof(line) - 1;

	/* ---------- write + sync ---------- */
	UINT bw = 0;
	FRESULT r = f_write(&Fil, line, pos, &bw);

	{
		float radioAlt = psiToM(prevPressure);
		uint8_t rpos = snprintf(radioPayload, sizeof(radioPayload), "%lu,%s",
							(unsigned long)HAL_GetTick(), stateNames[activeState]);

		// alt (3rd column)
		if (rpos < sizeof(radioPayload) - 1)
		{
			uint8_t rn = (!readError[4] && !isnan(radioAlt))
						 ? snprintf(radioPayload + rpos, sizeof(radioPayload) - rpos, ",%.2f", radioAlt)
						 : snprintf(radioPayload + rpos, sizeof(radioPayload) - rpos, ",-9999");
			if (rn > 0)
				rpos += rn;
		}

		// ax,ay,az,gx,gy,gz,t1,t2,p
		for (int i = 0; i < 9 && rpos < sizeof(radioPayload) - 1; i++)
		{
			uint8_t rn;
			if (good[i])
				rn = snprintf(radioPayload + rpos, sizeof(radioPayload) - rpos, ",%.*f", (i == 8) ? 4 : 2, vals[i]);
			else
				rn = snprintf(radioPayload + rpos, sizeof(radioPayload) - rpos, ",-99");
			if (rn > 0)
				rpos += rn;
		}

		if (rpos < 0)
			rpos = 0;
		if (rpos >= sizeof(radioPayload))
			rpos = sizeof(radioPayload) - 1;
		radio_len = rpos;
	}



	
	if (r != FR_OK || bw != (UINT)pos)
		sdErrorCount++;

	if (HAL_GetTick() - timeLastSync > 2500)
	{
		if (f_sync(&Fil) != FR_OK)
			sdErrorCount++;
		timeLastSync = HAL_GetTick();
	}

	return !readError[4];
}

float getAltFromLaunchM(float currentPSI)
{
	return psiToM(currentPSI) - LAUNCH_ALTITUDE_M;
}
float mtoft(float meters)
{
	return meters * 3.28084;
}
float psiToM(float psi)
{
	if (!(psi > 0.0f && psi < 20.0f))
		return NAN;
	return (1.0f - powf(psi / offsetPSI, 0.190284f)) * 44307.69f;
}
float calibrateOffsetPSI(float knownAltM, int CAL_NUM_READS, double alpha)
{
	for (int i = 0; i < CAL_NUM_READS; i++)
	{
		MPR_Read(&mprSensor);
		while (mprSensor.busy)
			;

		if (i == 0)
			prevPressure = mprSensor.pressure; // seed so the filter doesn't ramp from a stale value
		else
			exponentialFilter(mprSensor.pressure, &prevPressure, alpha);
	}

	return prevPressure / powf(1.0f - knownAltM / 44307.69f, 5.2553f);
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
