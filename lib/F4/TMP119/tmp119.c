#include "tmp119.h"

uint8_t rx[2];

void TMP119_setAddr(struct tmp119 *dev, bool addr2)
{
    if(addr2){
       dev->i2c_addr=TMP119_ADDR2;
    }
    else{
        dev->i2c_addr=TMP119_ADDR1;
    }
    
}
HAL_StatusTypeDef TMP119_StartRead(struct tmp119 *dev) {
    HAL_StatusTypeDef ret = HAL_BUSY;
    if (!dev->busy) {
        dev->busy = true;
        ret = HAL_I2C_Mem_Read_DMA(&hi2c2, dev->i2c_addr, TMP119_REG_TEMP, I2C_MEMADD_SIZE_8BIT, rx, 2);
       
    }
     if (ret != HAL_OK) dev->busy = false;
    return ret;
}

void TMP119_GetTemperature(struct tmp119 *dev) {
    dev->raw = (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    dev->temp = (float)dev->raw * 0.0078125f;
    dev->new_data = true;
    dev->busy = false;
}
void TMP119_SetLowPower(struct tmp119 *dev,bool lowPower){
    if(lowPower){
       // 0x0220 for default and 0x0620 for low power
        uint16_t config_val = 0x0620; 
        uint16_t config_swapped = __REV16(config_val);

        HAL_I2C_Mem_Write(&hi2c2, dev->i2c_addr,  0x01, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&config_swapped, 2, 1000);
    }
    if(!lowPower){
       // 0x0220 for default and 0x0620 for low power
        uint16_t config_val = 0x0220; 
        uint16_t config_swapped = __REV16(config_val);

        HAL_I2C_Mem_Write(&hi2c2,  dev->i2c_addr,  0x01, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&config_swapped, 2, 1000);
    }
}