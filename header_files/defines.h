/* 
 * File:   defines.h
 * Author: Fernando.Fernandes
 *
 * Created on August 19, 2022, 3:32 PM
 */

#ifndef DEFINES_H
#define	DEFINES_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// Defines /////////////////////////////////////////////////////////////////////    

// Digital Pins    
#define PIN_CAN_MODE            RA4
#define PIN_WDT_IN              RA5
#define PIN_3V3_AUX_POWER       RC0
#define PIN_IO_INTERRUPT        RC5    
#define PIN_HEADLIGHT           RB5

#define IO_6V_AUX_POWER         0
#define IO_TAIL_LIGHT           1
#define IO_MOTOR_A              2
#define IO_MOTOR_B              3
#define IO_ACCEL_INTERRUPT      4
#define IO_DRIVER_FAULT         5     
#define IO_HALL_VIOLATION       6       
#define IO_HALL_DOCK            7 
    
// Analog Pins
#define ADC_HALL_RIGHT_IN       0    
#define ADC_HALL_RIGHT_OUT      1   
#define ADC_HALL_LEFT_IN        2     
#define ADC_HALL_LEFT_OUT       3  
    
#define ADC_LIGHT_SENSOR        7    

// TIMES ///////////////////////////////////////////////////////////////////////
#define TIMER_BASE_100MS                    100 

#define TIME_WAIT_RELAY                     10   
    
//BIKE CONFIG
    
// GENERAL /////////////////////////////////////////////////////////////////////
#define ADC_SAMPLES                         10
    
#define MAX_VOLTAGE_CLOSE_CIRCUIT           100
#define MIN_VOLTAGE_OPEN_CIRCUIT            4000 
    


// STRUCTS ///////////////////////////////////////////////////////////////////// 
    
typedef struct
{
    bool dock_sensor;
    bool violation_sensor;   
    
    uint16_t hall_right_out;
    uint16_t hall_right_in;
    uint16_t hall_left_out;
    uint16_t hall_left_in;    
    
}Hall_MeasuresTypedef;

typedef struct
{    
    uint16_t time_can_message_alarm_low;
    uint16_t time_can_message_alarm_high;
    uint16_t time_can_message_alarm_ihm;    
    
    uint16_t time_can_message_fail_low;  
    uint16_t time_can_message_fail_high;  
    uint16_t time_can_message_fail_ihm;
    
    uint16_t time_can_message_tag;
    uint16_t time_can_message_status;
    
    uint16_t time_can_message_timeout;
    
    uint16_t time_can_message_beat;
    uint16_t time_can_message_serial;
    
    uint32_t beat_count;    
    
}CAN_MSG_ControlTypedef;

typedef struct
{    
    uint16_t time_ack;    
    uint16_t time_can_message1;
    uint16_t time_can_message2;
    uint16_t time_can_message3;
    uint16_t time_can_message4;
    uint16_t time_can_message5;    
        
}CAN_MSG_ValidationTypedef;

typedef struct
{    
    bool accelerometer_fail;    
    bool bike_docked;
    bool lock_violed;

    uint64_t my_id;
    uint32_t random_number;    
    
    uint8_t lock_last_command;
    
    uint8_t state_lock; 
    uint8_t state_motor;       
    uint8_t state_pin_left;
    uint8_t state_pin_right;  
    
    uint64_t rfid_current_tag;    
    
}TYPEDEF_LOCK_STATUS;

typedef struct
{    
    uint8_t type;
    
    uint8_t month;
    uint8_t year;
    
    uint32_t sequency_number;
    
}TYPEDEF_BIKE_SERIAL;

typedef struct
{
    uint8_t mode_iot;
    uint8_t mode_ldr;  
    
}TYPEDEF_BIKE_STATUS;  

typedef struct
{    
    uint8_t mode_lock_new;
    uint8_t mode_lock_current;    
    uint8_t mode_lock_internal;
        
    uint8_t config_head_percent;
    uint16_t config_head_pwm_value;   
    uint16_t config_head_min_adc;    
    
    uint16_t config_blink_head_on;
    uint16_t config_blink_head_off;
    
    uint16_t config_blink_tail_on;
    uint16_t config_blink_tail_off;
    
    uint16_t config_blink_pause_on;
    uint16_t config_blink_pause_off;    
    
    uint16_t config_time_alarm_on;
    uint16_t config_time_wait_release;

    uint8_t config_accel_threshold;    

}TYPEDEF_LOCK_CONFIG;

// ENUMS ///////////////////////////////////////////////////////////////////////    
    
enum EEPROM_Table
{
    ENUM_EEPROM_BOARD_APROVED,
    
    // Hall MEMORY  
    ENUM_EEPROM_HALL_VIOLATION,
    ENUM_EEPROM_HALL_DOCK,     
    
    // LEFT
    ENUM_EEPROM_HALL_LEFT_CLOSE_OUT_H,
    ENUM_EEPROM_HALL_LEFT_CLOSE_OUT_L,
    ENUM_EEPROM_HALL_LEFT_OPEN_IN_H,
    ENUM_EEPROM_HALL_LEFT_OPEN_IN_L, 
   
    // RIGHT    
    ENUM_EEPROM_HALL_RIGHT_CLOSE_IN_H,
    ENUM_EEPROM_HALL_RIGHT_CLOSE_IN_L,
    ENUM_EEPROM_HALL_RIGHT_OPEN_IN_H,       
    ENUM_EEPROM_HALL_RIGHT_OPEN_IN_L, 
    
    // PRODUCTION CONTROL
    // Type
    ENUM_EEPROM_SERIAL_TYPE,
    
    // SEQUENCY
    ENUM_EEPROM_SERIAL_SEQUENCY_1,
    ENUM_EEPROM_SERIAL_SEQUENCY_2,
    ENUM_EEPROM_SERIAL_SEQUENCY_3,
    ENUM_EEPROM_SERIAL_SEQUENCY_4,    
    
    // DATE
    ENUM_EEPROM_SERIAL_YEAR,
    ENUM_EEPROM_SERIAL_MONTH,    
};

enum LockState_Table
{
    ENUM_LOCK_STATE_OPEN = 1,
    ENUM_LOCK_STATE_CLOSE,
    ENUM_LOCK_STATE_INDEFINED,
};

enum TestState_Table
{
    ENUM_STAGE_1 = 0,
    ENUM_STAGE_2,
    ENUM_STAGE_3,
    ENUM_STAGE_4,
    ENUM_STAGE_5,
    ENUM_STAGE_6,
    ENUM_STAGE_7,
    
    ENUM_STAGE_SUCESS,
    ENUM_STAGE_FAIL,
};

enum LockCommand_Table
{
    ENUM_LOCK_COMMAND_OPEN = 1,
    ENUM_LOCK_COMMAND_CLOSE,
};

enum MotorState_Table
{
    ENUM_MOTOR_IDLE = 1,
    ENUM_MOTOR_OPENING, 
    ENUM_MOTOR_CLOSING,     
    ENUM_MOTOR_TIMEOUT,
};

enum PinsState_Table
{
    ENUM_PIN_CLOSE = 1,
    ENUM_PIN_OPEN, 
    ENUM_PIN_ERROR,
};

enum ActNextAct_Table
{
    ENUM_ACT_IDLE = 1,
    ENUM_ACT_OPEN,
    ENUM_ACT_CLOSE,    
};

enum LockMode_Table
{    
    ENUM_MODE_LOCK_LOCK = 1,
    ENUM_MODE_LOCK_WAIT_RELEASE,    
    ENUM_MODE_LOCK_UNLOCK,
    ENUM_MODE_LOCK_SELFTEST,
    ENUM_MODE_LOCK_DOCKING,
};

enum IotBikeMode_Table
{    
    ENUM_MODE_IOT_UNDEFINED = 0,
    ENUM_MODE_IOT_PLAY,
    ENUM_MODE_IOT_STOP,
    ENUM_MODE_IOT_PAUSE,
    ENUM_MODE_IOT_INOP,
    ENUM_MODE_IOT_TRANSPORT,    
};

enum LDRState_Table
{
    ENUM_MODE_LDR_DAY = 1,
    ENUM_MODE_LDR_NIGHT,        
};

#ifdef	__cplusplus
}
#endif

#endif	/* DEFINES_H */

