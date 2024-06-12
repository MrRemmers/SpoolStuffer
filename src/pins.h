//Pins.h//
#include <Arduino.h>

#define RXD2       GPIO_NUM_16
#define TXD2       GPIO_NUM_17

#define dataPinX   33
#define dataPinY   GPIO_NUM_25  //GPIO_NUM_25
#define clockPin   32
#define EN_PIN     GPIO_NUM_4   //enable (CFG6)
#define xDIR_PIN   GPIO_NUM_19  // X direction
#define xSTEP_PIN  GPIO_NUM_21  // X  step motor
#define yDIR_PIN   GPIO_NUM_22  // Y direction
#define ySTEP_PIN  GPIO_NUM_23 // Y step motor

#define valve_PIN  GPIO_NUM_2

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define X_CS_PIN         18 // Chip select
#define Y_CS_PIN          5 // Chip select

#define R_SENSE 0.11f //