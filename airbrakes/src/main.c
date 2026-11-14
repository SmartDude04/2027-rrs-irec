#include "main.h"
#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

#include "airbrake_config.h"
#include "airbrake_mpc.h"

#include "usb_device.h"
#include "usb_serial.h"
#include "fatfs.h"
#include "sdmmc.h"
#include "spi.h"
// #include "lora_sx1276.h"
// #include "radio.h"

static ab_mpc_t mpc;
// static lora_sx1276 lora;

/*
Basic State Machine Logic

Idle: Initial state on the pad
    Everything starts here.

Boost Phase: AIR BRAKES ARE NOT ACTIVE
    Significant acceleration is detected for more than 2 seconds and velocity
    has gone above 10 m/s

Coast Phase: AIR BRAKES ARE NOW ACTIVE
    Has been in the boost phase for 4 seconds, velo has gone below 130 m/s (426 ft/s)

Idle: apogee detected
    Acceleration has gone to 0 or have been in the coast phase for 13 seconds

Failure: Something goes wrong and everything stops
*/

enum RocketState {
    WaitingForLaunch,
    Armed,
    Boost,
    Coast,
    Idle,
    Failure,
};

enum RocketState state;

void SystemClock_Config(void);
void Error_Handler(void);
void MPU_Config(void);
void SDIO_SDCard_Test();

void run_mprls(void);
void airbrake_start(void);
void servo_set_opening(float a);
void airbrake_tick(float alt_m, float vz_mps);


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();

    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C1_DMA_Init();
    mprls_init();
  
    MPU_Config();
    MX_USB_DEVICE_Init();
    MX_SDMMC1_SD_Init();
    MX_FATFS_Init();
  
    HAL_Delay(5000);
    HAL_Delay(3000);
  
    SDIO_SDCard_Test();

    airbrake_start();
    state = WaitingForLaunch;

    /* Original test values; replace with measured altitude and velocity. */
    float h = 231.0f;
    float v = 140.0f;

    while (1)
    {
        run_mprls();

        /* Call every AB_DT_MPC seconds once timing is implemented. */
        airbrake_tick(h, v);
    }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

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

 /* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];

  BYTE work[512] __attribute__ ((aligned (4)));  /* Work area (larger is better for processing time) */
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, SDPath, 0);
    if (FR_Status != FR_OK)
    {
      SerialWrite("Error! While Mounting SD Card 0 , Error Code: (%i)\r\n", FR_Status);
      break;
    }
	FR_Status = f_mkfs(SDPath, FM_ANY, 0, work, sizeof(work));
  	if (FR_Status != FR_OK) 
	{
	  SerialWrite("Error! While Formatting SD Card, Error Code: (%i)\r\n", FR_Status);
	  break;
    }
	FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      SerialWrite("Error! While Mounting SD Card 1, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    SerialWrite("SD Card Mounted Successfully! \r\n\n");
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    SerialWrite("Total SD Card Size: %lu Bytes\r\n", TotalSize);
    SerialWrite("Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    // Open the file
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      SerialWrite("Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    SerialWrite("Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SDIO + DMA, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDIO + DMA, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      SerialWrite("Error! While Opening (MyTextFile.txt) File For Read.. \r\n");
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    SerialWrite("Data Read From (MyTextFile.txt) Using f_gets():%s", RW_Buffer);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    SerialWrite("Data Read From (MyTextFile.txt) Using f_read():%s", RW_Buffer);
    // Close The File
    f_close(&Fil);
    SerialWrite("File Closed! \r\n\n");
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      SerialWrite("Error! While Opening (MyTextFile.txt) File For Update.. \r\n");
      break;
    }
    // (2) Write New Line of Text Data To The File
    FR_Status = f_puts("This New Line Was Added During File Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    SerialWrite("Data Read From (MyTextFile.txt) After Update:\r\n%s", RW_Buffer);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    /*
    FR_Status = f_unlink(MyTextFile.txt);
    if (FR_Status != FR_OK){
        SerialWrite("Error! While Deleting The (MyTextFile.txt) File.. \r\n");
        USC_CDC_Print(TxBuffer);
    }
    */
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      SerialWrite("\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
  } else{
      SerialWrite("\r\nSD Card Un-mounted Successfully! \r\n");
  }
}



void run_mprls(void)
{
    if (mprls_status == IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();

        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF(
                "request_mprls_read() failed with code %d", ret
            );
            while (1);
        }
    }

    if (mprls_status == MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();

        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF(
                "start_mprls_dma_read() failed with code %d", ret
            );
            while (1);
        }
    }

    if (mprls_status == DATA_READY)
    {
        const uint32_t pressure =
            ((uint32_t)mprls_pressure_rx_buffer[1] << 16)
            | ((uint32_t)mprls_pressure_rx_buffer[2] << 8)
            | (uint32_t)mprls_pressure_rx_buffer[3];

        USB_CDC_PRINTF("Pressure: %lu\n", (unsigned long)pressure);

        /*
         * TODO: Convert raw pressure to altitude and estimate
         * vertical velocity, then pass those to airbrake_tick().
         */

        mprls_status = IDLE;
    }
}

void airbrake_start(void)
{
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
}

void servo_set_opening(float a)
{
    float angle = a * 70;           // Taken from dynamics, have 0-70 degrees to work with for deployment
    Servo_Move_Degrees((int)angle);
}

void airbrake_tick(float alt_m, float vz_mps)
{
    ab_info_t info;
    float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
    servo_set_opening(alpha);
}
