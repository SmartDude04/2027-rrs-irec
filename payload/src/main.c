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
struct tmp119 temp_sensor1;
struct tmp119 temp_sensor2;

enum STATE
{
	IDLE,
	ARMED,
	ASCENT,
	DESCENT,
	RECOVERY,
	ERRORSTATE
};
const char* stateNames[] = {
    "IDLE",
    "ARMED",
    "ASCENT",
    "DESCENT",
    "RECOVERY",
    "ERRORSTATE"
};
enum STATE activeState;
bool stateChangeComplete = false;
double prevPressure =0;
int startCounter =0;
uint32_t launchStart = 0;
int main()
{
	activeState = ASCENT;

	HAL_Init();
	SystemClock_Config();
	LED_Init();
	MX_USB_DEVICE_Init();
	I2C_Init(&hi2c1);
	gpio_init();
	HAL_Delay(2000);
	ICM_init(&icm_sensor);
	TMP119_setAddr(&temp_sensor1,false);
	TMP119_setAddr(&temp_sensor2,true);

	while (1)
	{

		switch (activeState)
		{
		case IDLE:
			if (!stateChangeComplete)
			{
				TMP119_SetLowPower(&temp_sensor1,true);
				TMP119_SetLowPower(&temp_sensor2,true);
				ICM_setLowPower(true);
				// mpr is low power by default when not reading
				stateChangeComplete = true;
			}

			// broken by radio command to "ARM" - switch to case ARMED
			// all but radio off
			// if () TODO: PUT RADIO TRIGGER HERE
			// {
				// stateChangeComplete = false;
				//activeState = ARMED;
			//}
			break;
		case ARMED:
			if (!stateChangeComplete)
			{
				ICM_setLowPower(false);
				stateChangeComplete = true;
			}
			if (!icm_sensor.aBusy&&!mprSensor.busy)
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
			if(icm_sensor.new_accel_data){
			SerialWrite("Accel, %f, Pressure, %.2f psi\n", icm_sensor.accelY, mprSensor.pressure);
			icm_sensor.new_accel_data = false;
			}
		
		}
		
		// broken by accellerometer detecting large spike - "launch", switch to case ASCENT
		// broken by sensor reading error  - something not recieving, radio com. failing, switch to case ERROR
		// imu on, accellerometer on, TMP off, pressure low reading?
		if (icm_sensor.accelY>10&&(abs(mprSensor.pressure-prevPressure))>10)// make sure this is right axis and change these numbers to be real
		{
			startCounter++;
			if(startCounter>=5){ // this is my thought of making sure that we don't get like problematic readings/ 1 reading says launch 
			stateChangeComplete = false;
			activeState = ASCENT;
			launchStart = HAL_GetTick();
		}
		prevPressure=mprSensor.pressure;
		}
		break;
	case ASCENT:
		if (!stateChangeComplete)
		{
			
			TMP119_SetLowPower(&temp_sensor1,false);
			TMP119_SetLowPower(&temp_sensor2,false);
			ICM_setLowPower(false);
			stateChangeComplete = true;
		}
		// make sure last loop is "done" before starting the next one
		if (!icm_sensor.aBusy &&
			!icm_sensor.gBusy &&
			!temp_sensor1.busy&&!temp_sensor2.busy&&!mprSensor.busy)
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
			while(mprSensor.busy)
			;
		}
		if (icm_sensor.new_accel_data &&
			icm_sensor.new_gyro_data &&
			temp_sensor1.new_data&&temp_sensor2.new_data)
		{
			uint32_t elapsed = HAL_GetTick() - launchStart;
			// this is where all the stuff is properly updated and should be referenced/called from
			SerialWrite("t:%u, | ax:%.2f,ay:%.2f,az:%.2f,| gx:%.2f,gy:%.2f,gz:%.2f, | t1:%.2f,t2:%.2f, | p:%.2f\n",
            (unsigned long)elapsed,
            icm_sensor.accelX, icm_sensor.accelY, icm_sensor.accelZ,
            icm_sensor.gyroX, icm_sensor.gyroY, icm_sensor.gyroZ,
            temp_sensor1.temp, temp_sensor2.temp, mprSensor.pressure);

			icm_sensor.new_accel_data = false;
			icm_sensor.new_gyro_data = false;
			temp_sensor1.new_data = false;
			temp_sensor2.new_data = false;
		}
		// launch timer begins?
		// broken by velocity switching to negative for >10 readings, baro reading concurring - apogee hit, switch to case DESCENT
		// all sensors on reading full
		//if () // dependant on velocity 
		//{
			//stateChangeComplete = false;
			//activeState = DESCENT;
		//}
		break;
	case DESCENT:
		if (!stateChangeComplete)
		{

			stateChangeComplete = true;
		}
		// broken by velocity going to ~0, baro reading going to ~ starting alt. Switch to case RECOVERY
		//if ()
		//{
			stateChangeComplete = false;
			activeState = RECOVERY;
		//}
		break;
	case RECOVERY:
		if (!stateChangeComplete)
		{

			stateChangeComplete = true;
		}
		// final state
		// all but radio off/save

		break;
	case ERRORSTATE:
		if (!stateChangeComplete)
		{

			stateChangeComplete = true;
		}

		break;
	}
	//SerialWrite("Current State: %s\n", stateNames[activeState]);
	HAL_Delay(10);
}}

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