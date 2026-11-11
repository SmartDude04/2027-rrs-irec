#include "I2C_DMA.h"

void Error_Handler(void);

DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
void I2C_Init(I2C_HandleTypeDef *hi2c) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
  
    /* DMA interrupt init */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    hi2c->Instance = I2C2;
hi2c->Init.ClockSpeed = 100000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(i2cHandle->Instance==I2C2)
    {
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C2 GPIO Configuration
        PB10    ------> I2C2_SCL
        PB3     ------> I2C2_SDA
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* SDA */
GPIO_InitStruct.Pin = GPIO_PIN_3;
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C2 clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();

        /* I2C2 DMA Init */
        /* I2C2_RX Init */
        hdma_i2c2_rx.Instance = DMA1_Stream2;
hdma_i2c2_rx.Init.Channel = DMA_CHANNEL_7;
hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_LOW;
hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK) {
    Error_Handler();
}

__HAL_LINKDMA(i2cHandle, hdmarx, hdma_i2c2_rx);

        /* I2C1_TX Init */
        hdma_i2c2_tx.Instance = DMA1_Stream7;
hdma_i2c2_tx.Init.Channel = DMA_CHANNEL_7;
hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_LOW;
hdma_i2c2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK) {
    Error_Handler();
}

__HAL_LINKDMA(i2cHandle, hdmatx, hdma_i2c2_tx);

        /* I2C1 interrupt Init */
       HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    /* USER CODE BEGIN I2C1_MspInit 1 */

    /* USER CODE END I2C1_MspInit 1 */
    }
}