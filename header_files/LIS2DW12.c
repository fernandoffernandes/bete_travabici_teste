/* 
 * File:   lis2dw12.c
 * Author: Fernando.Fernandes
 *
 * Created on 14 de Setembro de 2022, 10:14
 */

#include <xc.h>
#include "LIS2DW12.h"

bool Accel_Config(uint8_t threshold)
{
    printf("Accel_Config - Starting Accelerometer \r\n");    
    
    uint8_t value = Accel_ReadRegister(REG_WHO_AM_I); 
    printf("Accel_Config - Who I'm = 0x%x \r\n", value); 
    
    if(value != 0x44)
    {
        printf("Accel_Config - Accelerometer Fail \r\n");
        return false;
    }
    
    if(threshold > 63)
    {
        printf("Threshold Invalid - Max value = 63");
        threshold = 63;
    }
        
    // Turn on the accelerometer
    I2C1_Write1ByteRegister(AcellAddress, REG_CTRL1, 0x50);
    
    // Set duration for inactivity detection and wake-up detection
    I2C1_Write1ByteRegister(AcellAddress, REG_WAKE_UP_DUR, 0x42);
    
    // Set activity/inactivity threshold    
    threshold |= 0x40;
    I2C1_Write1ByteRegister(AcellAddress, REG_WAKE_UP_THS, threshold);

    // Activity (wakeup) interrupt driven to INT1 pin    
    I2C1_Write1ByteRegister(AcellAddress, REG_CTRL4_INT1_PAD_CTRL, 0x20);
    
    // Enable interrupts
    I2C1_Write1ByteRegister(AcellAddress, REG_CTRL7, 0x20);    
    I2C1_Write1ByteRegister(AcellAddress, REG_CTRL3, 0x12);

    // Finish
    printf("Accel_Config - Accelerometer Started \r\n");
    
    return true;
}

uint8_t Accel_ReadRegister(uint8_t reg)
{
    return I2C1_Read1ByteRegister(AcellAddress, reg);    
}

uint16_t Accel_ReadRegister16b(uint8_t reg)
{
    return (uint16_t)(I2C1_Read1ByteRegister(AcellAddress, reg + 1) << 8 |
                      I2C1_Read1ByteRegister(AcellAddress, reg));
}

uint8_t Accel_ClearInterrupt(void)
{
    return Accel_ReadRegister(REG_WAKE_UP_SRC);
}



