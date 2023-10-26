/* 
 * File:   tca9534.c
 * Author: Fernando.Fernandes
 *
 * Created on 19 de Agosto de 2022, 10:14
 */

#include <xc.h>
#include "tca9534.h"

void IO_ConfigPorts(IO_StructTypedef *ioStruct, uint8_t port)
{
    //Config Ports
    ioStruct->address = 0x20;
    ioStruct->inImage = 0x00;
    ioStruct->outImage = 0x00;
    ioStruct->ioConfig = port; 
    
    I2C1_Write1ByteRegister(ioStruct->address, IO_REG_CONFIG, ioStruct->ioConfig);  
}

void IO_SetPort(IO_StructTypedef *ioStruct, uint8_t port)
{
    ioStruct->outImage = port;    
    I2C1_Write1ByteRegister(ioStruct->address, IO_REG_OUTPUT, ioStruct->outImage);    
}

void IO_SetPin(IO_StructTypedef *ioStruct, uint8_t pin)
{
    if(pin > 7)
    {
        printf("ERROR - IO INVALID PIN %d\r\n", pin);
        while(1);       
    }   
    
    uint8_t mask = (uint8_t)(0x01 << pin);     
    ioStruct->outImage |= mask;
    
    I2C1_Write1ByteRegister(ioStruct->address, IO_REG_OUTPUT, ioStruct->outImage); 
}

void IO_ResetPin(IO_StructTypedef *ioStruct, uint8_t pin)
{
    if(pin > 7)
    {        
        printf("ERROR - IO INVALID PIN %d\r\n", pin);
        while(1);       
    }       
    
    uint8_t mask = (uint8_t)(~(0x01 << pin));    
    ioStruct->outImage &= mask;
    
    I2C1_Write1ByteRegister(ioStruct->address, IO_REG_OUTPUT, ioStruct->outImage);  
}

bool IO_ReadPin(IO_StructTypedef *ioStruct, uint8_t pin)
{
    if(pin > 7)
    {
        printf("ERROR - IO INVALID PIN \r\n");
        while(1);       
    }    
    
    uint8_t mask = (uint8_t)(0x01 << pin);      
    uint8_t registerValue = I2C1_Read1ByteRegister(ioStruct->address, IO_REG_INPUT);
    
    return (registerValue & mask) ? true
                                  : false;    
}

bool IO_ReadPort(IO_StructTypedef *ioStruct)
{   
    return (I2C1_Read1ByteRegister(ioStruct->address, IO_REG_INPUT));
}