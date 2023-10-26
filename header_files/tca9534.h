/* 
 * File:   tca9534.h
 * Author: Fernando.Fernandes
 *
 * Created on 19 de Agosto de 2022, 10:14
 */

#ifndef TCA9534_H
#define	TCA9534_H

#ifdef	__cplusplus
extern "C" {
#endif

// Include ////////////////////////////////////////////////////    
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
    
#include "../mcc_generated_files/examples/i2c1_master_example.h"
    
// Structs ////////////////////////////////////////////////////  
    
typedef struct 
{
    uint8_t address;    
    uint8_t ioConfig;    
    uint8_t outImage;
    uint8_t inImage;   
}IO_StructTypedef;

// Enumerators ////////////////////////////////////////////////

enum 
{
    IO_REG_INPUT = 0,
    IO_REG_OUTPUT,
    IO_REG_POLARITY_REGISTER,
    IO_REG_CONFIG,            
};    

// Prototypes /////////////////////////////////////////////////

void IO_ConfigPorts(IO_StructTypedef *ioStruct, uint8_t port);

bool IO_ReadPin(IO_StructTypedef *ioStruct, uint8_t pin);
bool IO_ReadPort(IO_StructTypedef *ioStruct);

void IO_SetPin(IO_StructTypedef *ioStruct, uint8_t pin);
void IO_SetPort(IO_StructTypedef *ioStruct, uint8_t port);

void IO_ResetPin(IO_StructTypedef *ioStruct, uint8_t pin);

#ifdef	__cplusplus
}
#endif

#endif	/* TCA9534_H */

