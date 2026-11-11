#include "usb_device.h"
#include "usb_serial.h"
#include "tmp119.h"
#include "I2C_DMA.h"
#include "gpio.h"
#include "lora_sx1276.h"
#include "SPI.h"
#include "radio.h"
#include "sdio.h"
#include "fatfs.h"


void SystemClock_Config();
void Error_Handler();
void SDIO_SDCard_Test();

// I2C_HandleTypeDef hi2c1;
// struct tmp119 temp_sensor;

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
	.spi_timeout = LORA_DEFAULT_SPI_TIMEOUT
};
struct radio_link link;

int main() {
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    MX_USB_DEVICE_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	SPI_Init();
	radio_link_init(&lora, &link, NODE_PAYLOAD);
	HAL_Delay(5000);

    while (1) {

	}
		
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
    }
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