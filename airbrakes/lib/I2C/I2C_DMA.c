#include "I2C_DMA.h"

static DMA_HandleTypeDef i2c1_tx_dma_handle = {0};
static DMA_HandleTypeDef i2c1_rx_dma_handle = {0};
static DMA_HandleTypeDef i2c2_tx_dma_handle = {0};
static DMA_HandleTypeDef i2c2_rx_dma_handle = {0};

I2C_HandleTypeDef i2c1_handle = {0};
I2C_HandleTypeDef i2c2_handle = {0};

static void init_dma(DMA_HandleTypeDef *dma, DMA_Stream_TypeDef *stream, const uint32_t request, const uint32_t direction)
{
    dma->Instance = stream;
    dma->Init.Request = request;
    dma->Init.Direction = direction;
    dma->Init.PeriphInc = DMA_PINC_DISABLE;
    dma->Init.MemInc = DMA_MINC_ENABLE;
    dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma->Init.Mode = DMA_NORMAL;
    dma->Init.Priority = DMA_PRIORITY_LOW;
    dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(dma) != HAL_OK)
    {
        while (1);
    }
}

static void enable_irq(const IRQn_Type irq)
{
    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
}

static void init_i2c(I2C_HandleTypeDef *hi2c)
{
    // Retains the existing timing; actual speed depends on the I2C clock.
    hi2c->Init.Timing = 0x00707CBB;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        while (1);
    }

    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        while (1);
    }

    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
    {
        while (1);
    }
}

// Retain the existing name so callers do not need to change.
void I2C1_DMA_Init(void)
{
    i2c1_handle.Instance = I2C1;
    i2c2_handle.Instance = I2C2;

    init_i2c(&i2c1_handle);
    init_i2c(&i2c2_handle);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL || (hi2c->Instance != I2C1 && hi2c->Instance != I2C2))
    {
        return;
    }

    // Per alternate function table in the datasheet, the following pins are I2C1 interfaces (listed as SCL & SDA):
    // - PB6 & PB7
    // - PB8 & PB9
    // We will be using PB6 & PB7 for now
    // The following  pins are I2C2 interfaces (listed as SCL & SDA):
    // - PB10 & PB11
    // We will be using PB10 and PB11
    
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &gpio);

        init_dma(&i2c1_tx_dma_handle, DMA1_Stream1, DMA_REQUEST_I2C1_TX, DMA_MEMORY_TO_PERIPH);
        init_dma(&i2c1_rx_dma_handle, DMA1_Stream0, DMA_REQUEST_I2C1_RX, DMA_PERIPH_TO_MEMORY);

        __HAL_LINKDMA(hi2c, hdmatx, i2c1_tx_dma_handle);
        __HAL_LINKDMA(hi2c, hdmarx, i2c1_rx_dma_handle);

        enable_irq(I2C1_EV_IRQn);
        enable_irq(I2C1_ER_IRQn);
        enable_irq(DMA1_Stream0_IRQn);
        enable_irq(DMA1_Stream1_IRQn);
    }
    else
    {
        __HAL_RCC_I2C2_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &gpio);

        init_dma(&i2c2_tx_dma_handle, DMA1_Stream3, DMA_REQUEST_I2C2_TX, DMA_MEMORY_TO_PERIPH);
        init_dma(&i2c2_rx_dma_handle, DMA1_Stream2, DMA_REQUEST_I2C2_RX, DMA_PERIPH_TO_MEMORY);

        __HAL_LINKDMA(hi2c, hdmatx, i2c2_tx_dma_handle);
        __HAL_LINKDMA(hi2c, hdmarx, i2c2_rx_dma_handle);

        enable_irq(I2C2_EV_IRQn);
        enable_irq(I2C2_ER_IRQn);
        enable_irq(DMA1_Stream2_IRQn);
        enable_irq(DMA1_Stream3_IRQn);
    }
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c1_rx_dma_handle);
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c1_tx_dma_handle);
}

void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c2_rx_dma_handle);
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&i2c2_tx_dma_handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c1_handle);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c1_handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c2_handle);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c2_handle);
}