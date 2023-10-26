/* 
 * File:   LIS2DW12.h
 * Author: Fernando
 *
 * Created on September 14, 2022, 2:07 PM
 */

#ifndef LIS2DW12_H
#define	LIS2DW12_H

#ifdef	__cplusplus
extern "C" {
#endif

// Include ////////////////////////////////////////////////////    
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
    
#include "../mcc_generated_files/examples/i2c1_master_example.h"

// Defines ////////////////////////////////////////////////////
    
#define AcellAddress                0x18    

#define REG_OUT_T_L                 0x0D
#define REG_OUT_T_H                 0x0E
    
#define REG_WHO_AM_I                0x0F 
#define REG_CTRL1                   0x20
#define REG_CTRL2                   0x21 
#define REG_CTRL3                   0x22  
#define REG_CTRL4_INT1_PAD_CTRL     0x23  
#define REG_CTRL5_INT2_PAD_CTRL     0x24 

#define REG_CTRL6                   0x25   
#define REG_OUT_T                   0x26
#define REG_STATUS                  0x27
#define REG_OUT_X_L                 0x28 
#define REG_OUT_X_H                 0x29

#define REG_OUT_Y_L                 0x2A     
#define REG_OUT_Y_H                 0x2B      
#define REG_OUT_Z_L                 0x2C       
#define REG_OUT_Z_H                 0x2D       

#define REG_FIFO_CTRL               0x2E      
#define REG_FIFO_SAMPLES            0x2F        
#define REG_TAP_THS_X               0x30         
#define REG_TAP_THS_Y               0x31    
#define REG_TAP_THS_Z               0x32
#define REG_INT_DUR                 0x33 
#define REG_WAKE_UP_THS             0x34       
#define REG_WAKE_UP_DUR             0x35
#define REG_FREE_FALL               0x36     

#define REG_STATUS_DUP              0x37       
#define REG_WAKE_UP_SRC             0x38      
#define REG_TAP_SRC                 0x39         
#define REG_SIXD_SRC                0x3A       
#define REG_ALL_INT_SRC             0x3B 

#define REG_X_OFS_USR               0x3C       
#define REG_Y_OFS_USR               0x3D       
#define REG_Z_OFS_USR               0x3E     
#define REG_CTRL7                   0x3F     
    
// Structs ////////////////////////////////////////////////////     

// Enumerators ////////////////////////////////////////////////  
    
// Prototypes ///////////////////////////////////////////////// 
    
bool Accel_Config(uint8_t threshold);
uint8_t Accel_ClearInterrupt(void);
uint8_t Accel_ReadRegister(uint8_t reg);
uint16_t Accel_ReadRegister16b(uint8_t reg); 
    
#ifdef	__cplusplus
}
#endif

#endif	/* LIS2DW12_H */

