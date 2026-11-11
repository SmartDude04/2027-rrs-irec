#include "I2C_DMA.h"

static DMA_HandleTypeDef i2c_tx_dma_handle = {0};
static DMA_HandleTypeDef i2c_rx_dma_handle = {0};

I2C_HandleTypeDef i2c1_handle = {0};

void I2C1_DMA_Init()
{
    i2c1_handle.Instance = I2C1;
    i2c1_handle.Init.Timing = 0x00707CBB; // 400 KHz mode for I2C interface
    
    if (HAL_I2C_Init(&i2c1_handle) != HAL_OK)
    {
        while (1);
    }
    
    // Configure analog and digital noise filters
    if (HAL_I2CEx_ConfigAnalogFilter(&i2c1_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        while (1);
    }
    
    if (HAL_I2CEx_ConfigDigitalFilter(&i2c1_handle, 0) != HAL_OK)
    {
        while (1);
    }
}

/**
 * Function must be implemented by the user per STM32H7xx HAL description
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef * hi2c)
{
    if (hi2c == NULL || hi2c->Instance != I2C1)
    {
        return;
    }
    
    // Per alternate function table in the datasheet, the following pins are I2C1 interfaces (listed as SCL & SDA):
    // - PB6 & PB7
    // - PB8 & PB9
    // We will be using PB6 & PB7 for now
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    // Configure PB6 and PB7 for alternate function open drain
    GPIO_InitTypeDef pb6_7_init = {0};
    pb6_7_init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    pb6_7_init.Mode = GPIO_MODE_AF_OD;
    pb6_7_init.Pull = GPIO_NOPULL;
    pb6_7_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pb6_7_init.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &pb6_7_init);
    
    // Enable NVIC for interrupt handling
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    
    // Initialize DMA for this I2C bus
    i2c_tx_dma_handle.Instance = DMA1_Stream1;
    i2c_tx_dma_handle.Init.Request = DMA_REQUEST_I2C1_TX;
    i2c_tx_dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    i2c_tx_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    i2c_tx_dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    i2c_tx_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2c_tx_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2c_tx_dma_handle.Init.Mode = DMA_NORMAL;
    i2c_tx_dma_handle.Init.Priority = DMA_PRIORITY_LOW;
    i2c_tx_dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    if (HAL_DMA_Init(&i2c_tx_dma_handle) != HAL_OK)
    {
        while (1);
    }
    
    __HAL_LINKDMA(hi2c, hdmatx, i2c_tx_dma_handle);
    
    i2c_rx_dma_handle.Instance = DMA1_Stream0;
    i2c_rx_dma_handle.Init.Request = DMA_REQUEST_I2C1_RX;
    i2c_rx_dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    i2c_rx_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    i2c_rx_dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    i2c_rx_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2c_rx_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2c_rx_dma_handle.Init.Mode = DMA_NORMAL;
    i2c_rx_dma_handle.Init.Priority = DMA_PRIORITY_LOW;
    i2c_rx_dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    if (HAL_DMA_Init(&i2c_rx_dma_handle) != HAL_OK)
    {
        while (1);
    }
    
    __HAL_LINKDMA(hi2c, hdmarx, i2c_rx_dma_handle);
    
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

// Below are all the interrupts this I2C bus will use
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c_rx_dma_handle);
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c_tx_dma_handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c1_handle);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c1_handle);
}