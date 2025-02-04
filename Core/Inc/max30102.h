/*
 * max30102.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Robert
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_


/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Defines */
/* Pulse detection parameters */
#define MAX30102_SAMPLES_PER_BURST 30 // max of 32, minimum of 1

#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000//2000
#define PULSE_GO_DOWN_THRESHOLD     1

#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size

/* SpO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     4

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500

#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT          MAX30100_LED_CURRENT_27_1MA



#define MAX30102_ADDRESS 0x57   // MAX30102 I2C address

// MAX30102 registers
#define REG_PART_ID 0xFF        // Device ID
#define REG_MODE_CONFIG 0x09    // Mode configuration
#define REG_SPO2_CONFIG 0x0A    // SPO2 configuration
#define REG_LED1_PA 0x0C        // LED1 (Red) pulse amplitude
#define REG_LED2_PA 0x0D        // LED2 (Infrared) pulse amplitude
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07      // FIFO data register
#define REG_INT1_EN 0x02    // interrupt enable register 1 of 2
#define REG_FIFO_CONFIG 0x08    // fifo configuration register
#define REG_TEMP_INTEGER		0x1F
#define REG_TEMP_FRACTION		0x20
#define REG_TEMP_CONFIG			0x21

#define BIT_EN_A_FULL_INT 6 // bit for generating interrupt on the fifo almost full flag
#define BIT_FIFO_A_FULL_VAL 0 // 3:0, value for generating almost full interrupt, 0 means FIFO fully utilized
#define BIT_MODE 0
#define BIT_SPO2_ADC_RGE 5
#define BIT_SPO2_SR 2
#define BIT_LED_PW 0

#define RED_LED	1
#define IR_LED	2

extern I2C_HandleTypeDef hi2c1;

typedef struct{
	uint32_t redLedRaw;
	uint32_t irLedRaw;
}FIFO_LED_DATA;

typedef enum {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PULSE_STATE_MACHINE;

typedef struct {
  bool pulseDetected;
  float heartBPM;
  float irCardiogram;
  float irDcValue;
  float redDcValue;
  float SpO2;
  uint32_t lastBeatThreshold;
  float dcFilteredIR;
  float dcFilteredRed;
  float temperature;
}MAX30102;

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;
/* USER CODE END Defines */

/* USER CODE BEGIN FP */
HAL_StatusTypeDef MAX30102_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX30102_ReadRegister(uint8_t reg, uint8_t *value);
uint8_t MAX30102_ReadFIFO(uint8_t *buffer, uint8_t length);
uint8_t MAX30102_Init(void);
uint8_t MAX30102_FIFO_Reset(void);
uint8_t MAX30102_setLedCurrent(uint8_t led, float currentLevel);
uint8_t MAX30102_DumpFifo(void);
void MAX30102_ProcessData(void);
float MAX30102_readTemperature(void);
MAX30102 pulseOximeter_update(FIFO_LED_DATA m_fifoData);
bool detectPulse(float sensor_value);
void balanceIntesities(float redLedDC, float IRLedDC);
float MAX30102_getBPM(void);
float MAX30102_getSPO2(void);
/* USER CODE END FP */

#endif /* INC_MAX30102_H_ */
