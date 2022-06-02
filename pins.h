#ifndef _ROBOHEART_PINS_H
#define _ROBOHEART_PINS_H
/*
*** This file will disapear and it will be replaced by an arduino_pin.h that we include into the esspresif arduino core
*/

//MOTOR 0
#define GPIO_M0A_PH_IN1     25     //  PHASE/IN1  //DAC1
#define GPIO_M0B_EN_IN2     26     //  ENABLE/IN2  //DAC2
#define GPIO_M0_MODE        2   //  MODE        
#define GPIO_M0_SLEEP       36    //  SLEEP

//MOTOR 1
#define GPIO_M1A_PH_IN1     27   //  PHASE/IN1 
#define GPIO_M1B_EN_IN2     32      //  ENABLE/IN2  
#define GPIO_M1_MODE        34    //  MODE       
#define GPIO_M1_SLEEP       35     //  SLEEP       

//MOTOR 2
#define GPIO_M2A_PH_IN1     33   //  PHASE/IN1
#define GPIO_M2B_EN_IN2     4   //  ENABLE/IN2
#define GPIO_M2_MODE        16    //  MODE 
#define GPIO_M2_SLEEP       17    //  SLEEP

//LOAD SWITCH ENABLE FOR MOTORS
#define LS_EN_CNTRL 14

#define I2C_SDA 21
#define I2C_SCL 22


//FOR REFERENCE: PWM PINS OF ESP32: 2, 4, 5, 12-19, 21-23, 27, 32, 33

#define MPU6050_I2C_ADDR 0x69

#endif