#ifndef _ROBOHEART_PINS_H
#define _ROBOHEART_PINS_H
/*
*** This file will disapear and it will be replaced by an arduino_pin.h that we
*include into the esspresif arduino core
*/

// MOTOR 0
#define GPIO_MA_PH_IN1 25  //  PHASE/IN1  //DAC1
#define GPIO_MA_EN_IN2 26  //  ENABLE/IN2  //DAC2
#define GPIO_MA_MODE 2     //  MODE
#define GPIO_MA_SLEEP 0    //  nSLEEP

// MOTOR 1
#define GPIO_MB_PH_IN1 27  //  PHASE/IN1
#define GPIO_MB_EN_IN2 32  //  ENABLE/IN2
#define GPIO_MB_MODE 2     //  MODE
#define GPIO_MB_SLEEP 0    //  nSLEEP

// MOTOR 2
#define GPIO_MC_PH_IN1 33  //  PHASE/IN1
#define GPIO_MC_EN_IN2 4   //  ENABLE/IN2
#define GPIO_MC_MODE 2     //  MODE
#define GPIO_MC_SLEEP 0    //  nSLEEP

// LOAD SWITCH ENABLE FOR MOTORS
#define LS_EN_CNTRL 14

#define I2C_SDA 21
#define I2C_SCL 22

// FOR REFERENCE: PWM PINS OF ESP32: 2, 4, 5, 12-19, 21-23, 27, 32, 33

#define MPU6050_I2C_ADDR 0x69

#define LED_BUILTIN 0

#define RX1 16
#define TX1 17

#define SPI_SS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18

#endif