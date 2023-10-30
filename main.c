/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F26K83
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "header_files/tca9534.h"
#include "header_files/defines.h"

// Prototypes ////////////////////////////////////////////////
void control_wdt(void);

uint16_t fx_eeprom_read_int16(uint16_t address);
void fx_eeprom_write_int16(uint16_t address, uint16_t data);
uint32_t fx_eeprom_read_int32(uint16_t address);
void fx_eeprom_write_int32(uint16_t address, uint32_t data);

void fx_system_start(void);
void control_system(void);

void fx_led_green_fast(void);
void fx_led_green_slow(void);

void fx_read_adc(void);
bool fx_read_button(void);
void fx_led_show_fail(uint8_t fail_code);

// Macros ////////////////////////////////////////////////////
#define POWER_SUPPLY_LOW_ON     IO_RC0_SetLow()
#define POWER_SUPPLY_LOW_OFF    IO_RC0_SetHigh()

#define POWER_SUPPLY_HIGH_ON    IO_ResetPin(&g_io_expander1, IO_6V_AUX_POWER)
#define POWER_SUPPLY_HIGH_OFF   IO_SetPin(&g_io_expander1, IO_6V_AUX_POWER)

#define LED_GREEN_ON            IO_RA2_SetLow()
#define LED_GREEN_OFF           IO_RA2_SetHigh()
#define LED_GREEN_BLINK         IO_RA2_Toggle()

#define LED_RED_ON              IO_RA3_SetLow()
#define LED_RED_OFF             IO_RA3_SetHigh()
#define LED_RED_BLINK           IO_RA3_Toggle()

#define RELAY_POSITIVE_ON       IO_SetPin(&g_io_expander1, 1)
#define RELAY_POSITIVE_OFF      IO_ResetPin(&g_io_expander1, 1)

#define RELAY_NEGATIVE_ON       IO_RC5_SetHigh()
#define RELAY_NEGATIVE_OFF      IO_RC5_SetLow()

// Variables /////////////////////////////////////////////////
// Structs
CAN_MSG_ControlTypedef g_can_messages;
CAN_MSG_ValidationTypedef g_can_validation_messages;

IO_StructTypedef g_io_expander1;

Hall_MeasuresTypedef g_hall_start;
Hall_MeasuresTypedef g_hall_current;

TYPEDEF_LOCK_CONFIG g_lock_config;
TYPEDEF_LOCK_STATUS g_lock_status;
TYPEDEF_BIKE_SERIAL g_lock_serial;
TYPEDEF_BIKE_STATUS g_bike_status;

// Flags
bool g_flag_timer_1s = false;
bool g_flag_timer_100ms = false;

// General 
uint8_t g_fail_code = 0;
uint8_t g_state_machine = 0;

uint16_t g_adc_1 = 0;
uint16_t g_adc_2 = 0;

// Times
uint16_t g_wait_time = 0;

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
    // Disable the Global Interrupts
    INTERRUPT_GlobalInterruptDisable();
    
    fx_system_start();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();  

    while(1)
    {
        control_wdt(); 
        control_system();
        
        // Debug        
        static bool flag_debug = 0;
        
        if(flag_debug != g_flag_timer_1s)
        {
            flag_debug = g_flag_timer_1s;        
            
            printf("state machine %d \r\n", g_state_machine); 
            
            fx_read_adc();
            printf("adc1=%d, adc2=%d \r\n", g_adc_1, g_adc_2);            
        }      
    }
}

// Control Functions ////////////////////////////////////////////////////
void control_system(void)
{
    switch (g_state_machine)
    {
        // Wait button        
        case ENUM_STAGE_1:
        {
            LED_RED_OFF;
            fx_led_green_slow();
            
            if(true == fx_read_button())
            {
                g_state_machine++;
                printf("Button detected \r\n");
            }
        }
        break; 
        
        // Check frame connection        
        case ENUM_STAGE_2:
        {
            if(false == IO_RC3_GetValue())
            {
                printf("Frame connected \r\n");                
                g_state_machine++;                
            }
            else
            {
                printf("Frame not connected \r\n");                
                
                g_fail_code = ENUM_FAIL_FRAME_CONNECTION;                
                g_state_machine = ENUM_STAGE_FAIL;                
            }            
        }
        break;
        
        // Turn on relay        
        case ENUM_STAGE_3:
        {
            printf("Relay positive powered \r\n");
            RELAY_POSITIVE_ON;
            
            g_wait_time = 15;           //*100ms                                  
            g_state_machine++;              
        }
        break;
        
        // Detect short between contact and frame
        case ENUM_STAGE_4:
        {
            fx_led_green_fast();

            // Detect short circuit with frame            
            if(0 == g_wait_time)
            {
                fx_read_adc();                
                
                if((g_adc_1 > MIN_VOLTAGE_OPEN_CIRCUIT) && 
                   (g_adc_2 > MIN_VOLTAGE_OPEN_CIRCUIT))
                {
                    printf("Short circuit not detected \r\n");
                    g_state_machine++;                    
                }
                else
                {
                    printf("Short circuit with frame detected \r\n");
                    
                    g_fail_code = ENUM_FAIL_SHORT_FRAME;
                    g_state_machine = ENUM_STAGE_FAIL;                    
                }                
            }            
        }
        break; 
        
        // Turn on relay
        case ENUM_STAGE_5:
        {
            printf("Relay negative powered \r\n");            
            RELAY_NEGATIVE_ON; 
            
            g_wait_time = 100;                                   
            g_state_machine++;            
        }
        break;         
        
        // Measure contact voltage
        case ENUM_STAGE_6:
        {            
            fx_led_green_fast();
            
            if(0 == g_wait_time)
            {
                fx_read_adc();                
                
                if((g_adc_1 < MAX_VOLTAGE_CLOSE_CIRCUIT) && 
                   (g_adc_2 < MAX_VOLTAGE_CLOSE_CIRCUIT))
                {
                    printf("Contact voltage OK!! \r\n");
                    g_state_machine++;                    
                }
                else
                {
                    printf("Contact voltage high detected \r\n");
                    
                    g_fail_code = ENUM_FAIL_HIGH_VOLTAGE;
                    g_state_machine = ENUM_STAGE_FAIL;
                }                
            }     
        }
        break; 
        
        case ENUM_STAGE_7:
        {
            printf("Relays OFF \r\n");
            
            g_wait_time = 50;            
            RELAY_POSITIVE_OFF;
            RELAY_NEGATIVE_OFF;
            
            g_state_machine = ENUM_STAGE_SUCESS;        
        }
        break;   
        
        case ENUM_STAGE_SUCESS:
        {
            LED_RED_OFF;
            LED_GREEN_ON;            
            
            if(0 == g_wait_time)
            {
                LED_RED_OFF;
                LED_GREEN_OFF;
                g_state_machine = ENUM_STAGE_1;
            }            
        }
        break;

        case ENUM_STAGE_FAIL:
        {
            static bool flag_fail = false;
            
            if(false == flag_fail)
            {
                flag_fail = true;
                
                LED_GREEN_OFF;            
                RELAY_POSITIVE_OFF;
                RELAY_NEGATIVE_OFF;                
                fx_led_show_fail(0);                
            }            
            
            fx_led_show_fail(g_fail_code);
            
            if(true == fx_read_button())
            {
                flag_fail = false;
                g_state_machine = ENUM_STAGE_WAIT_BT;                
            }
        }
        break;
        
        case ENUM_STAGE_WAIT_BT:
        {
            if(false == fx_read_button())
            {
                g_state_machine = ENUM_STAGE_1;                
            }            
        }
        break;
    }    
}

void control_wdt(void)
{        
    static bool last_flag = false;
    
    if(g_flag_timer_100ms != last_flag)
    {
        last_flag = g_flag_timer_100ms;        
        
        // EXTERNAL WDT
        IO_RA5_Toggle();
        
        // INTERNAL WDT         
        CLRWDT();
    }        
}


// Auxiliary Functions ////////////////////////////////////////////////////
uint16_t fx_eeprom_read_int16(uint16_t address)
{
    uint16_t retValue = (uint16_t)DATAEE_ReadByte(address) << 8 | 
                                  DATAEE_ReadByte(address + 1);    
    
    return retValue;
}

uint32_t fx_eeprom_read_int32(uint16_t address)
{
    uint32_t retValue = (uint32_t)fx_eeprom_read_int16(address) << 16 | 
                                  fx_eeprom_read_int16(address + 2);    
    
    return retValue;
}

void fx_eeprom_write_int16(uint16_t address, uint16_t data)
{
    DATAEE_WriteByte(address, (data >> 8));
    DATAEE_WriteByte(address + 1, (data & 0xFF));    
}

void fx_eeprom_write_int32(uint16_t address, uint32_t data)
{
    fx_eeprom_write_int16(address, (data >> 16));
    fx_eeprom_write_int16(address + 2, (data & 0xFFFF));
}

void fx_read_adc(void)
{
    uint16_t adc_1 = 0;
    uint16_t adc_2 = 0;    
    
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) 
    {
        adc_1 += ADCC_GetSingleConversion(0);
        adc_2 += ADCC_GetSingleConversion(1);
    }

    g_adc_1 = adc_1 / ADC_SAMPLES;
    g_adc_2 = adc_2 / ADC_SAMPLES;    
}

bool fx_read_button(void)
{
    static bool button_pressed = false;
    static uint16_t counter = 0;    
    
    if(false == IO_RC4_GetValue())
    {
        if(counter > 100)
        {
            button_pressed = true;            
        }            
        else
        {
            counter++; 
        }                                   
    }
    else
    {
        counter = 0; 
    
        if(true == button_pressed)
        {
            button_pressed = false;            
            return true; 
        }                
    }    
    
    return false;    
}

void fx_led_green_fast(void)
{
    static bool flag_led = false;
    
    if(flag_led != g_flag_timer_100ms)
    {
        flag_led = g_flag_timer_100ms;                
        LED_GREEN_BLINK;
    }    
}

void fx_led_green_slow(void)
{
    static bool flag_led = false;
    
    if(flag_led != g_flag_timer_1s)
    {
        flag_led = g_flag_timer_1s;                
        LED_GREEN_BLINK;
    }    
}

void fx_led_show_fail(uint8_t fail_code)
{
    // DEFINES /////////////////////////////////////////////////////////////////
    #define TIME_FAIL_LED_ON        10         //*100ms   
    #define TIME_FAIL_LED_OFF       10         //*100ms
    #define TIME_FAIL_NEXT_CODE     30         //*100ms 
    
    // VARIABLES ///////////////////////////////////////////////////////////////    
    static bool flag_led = false;
    static bool flag_time = false;    
    
    static uint16_t time_led = 0;
    static uint16_t blink_counter = 0;    
    
    // LOGIC ///////////////////////////////////////////////////////////////////    
    // Control reset 
    if(0 == fail_code)
    {
        flag_led = false;
        
        time_led = 0;        
        blink_counter = 0;
        
        return;
    }    
    
    // Control time
    if(flag_time != g_flag_timer_100ms)
    {
        flag_time = g_flag_timer_100ms;        
        
        if(time_led)
            time_led--;        
    }
        
    // Control Led    
    if(0 == time_led)
    {     
        flag_led = !flag_led;        
        
        if(false == flag_led)
        {
            LED_RED_ON;
            blink_counter++;
            time_led = TIME_FAIL_LED_ON;        
        }
        else
        {
            LED_RED_OFF;    

            if(blink_counter < fail_code)
            {
                time_led = TIME_FAIL_LED_OFF;                                
            }
            else
            {
                blink_counter = 0;
                time_led = TIME_FAIL_NEXT_CODE;
            }
        }    
    }
}

void fx_system_start(void)
{
    printf("Start System - Starting \r\n"); 
    
    printf("Start System - EEPROM \r\n");
    
    printf("Start System - IO Expander \r\n");
    IO_ConfigPorts(&g_io_expander1, 0x00); 
    
    printf("Start System - Reloading Default Values \r\n");
    
    printf("Start System - Starting Pins \r\n");
    LED_RED_OFF;
    LED_GREEN_OFF;
    RELAY_NEGATIVE_OFF;
    RELAY_POSITIVE_OFF;
    
    printf("Start System - Starting PS / PS \r\n");
    POWER_SUPPLY_LOW_ON;
    POWER_SUPPLY_HIGH_ON;    
    
    printf("Start System - Finished \r\n");    
}

/**
 End of File
*/