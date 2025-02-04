/*
 * max30102.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Robert
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

/* USER CODE BEGIN Defines */

#define MAX30102_SAMPLES_PER_BURST 30 // max of 32, minimum of 1
#define MAX30102_SAMPLE_RATE 50 // Hz

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

#define BIT_EN_A_FULL_INT 6 // bit for generating interrupt on the fifo almost full flag
#define BIT_FIFO_A_FULL_VAL 0 // 3:0, value for generating almost full interrupt, 0 means FIFO fully utilized
#define BIT_MODE 0
#define BIT_SPO2_ADC_RGE 5
#define BIT_SPO2_SR 2
#define BIT_LED_PW 0



extern I2C_HandleTypeDef hi2c1;

/* USER CODE END Defines */

/* USER CODE BEGIN 0 */
// I2C write to register
HAL_StatusTypeDef MAX30102_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, (MAX30102_ADDRESS << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// I2C read from register
HAL_StatusTypeDef MAX30102_ReadRegister(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, (MAX30102_ADDRESS << 1), reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

// Read FIFO data
uint8_t MAX30102_ReadFIFO(uint8_t *buffer, uint8_t length) {
    return HAL_I2C_Mem_Read(&hi2c1, (MAX30102_ADDRESS << 1), REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, buffer, length, 100);
}

// MAX30102 initialization
uint8_t MAX30102_Init(void) {
    uint8_t part_id = 0;

    // Read MAX30102 ID to verify device presence
    if (MAX30102_ReadRegister(REG_PART_ID, &part_id) != HAL_OK || part_id != 0x15) {
        return 0;  // Device not found
    }

    // Reset MAX30102
    MAX30102_WriteRegister(REG_MODE_CONFIG, 0b01000000);  // Reset MAX30102  (bit 6)
    HAL_Delay(10);

    // Configure SPO2 mode
    MAX30102_WriteRegister(REG_MODE_CONFIG, 0b011 << BIT_MODE);  // Use heart rate mode only (or 0x07 to enable SPO2)
    uint8_t spo2_bits = (0b01 << BIT_SPO2_ADC_RGE) |  (0b000 << BIT_SPO2_SR) | (0b11 << BIT_LED_PW);
    MAX30102_WriteRegister(REG_SPO2_CONFIG, spo2_bits);  // Set ADC range and sampling rate

    // Set LED brightness (0x24 represents medium brightness, adjustable)
    MAX30102_WriteRegister(REG_LED1_PA, 0x24);  // LED1 (Red)
    MAX30102_WriteRegister(REG_LED2_PA, 0x24);  // LED2 (Infrared)

    // interrupts
    MAX30102_WriteRegister(REG_FIFO_CONFIG, (32 - MAX30102_SAMPLES_PER_BURST) << BIT_FIFO_A_FULL_VAL);
    MAX30102_WriteRegister(REG_INT1_EN, 1 << BIT_EN_A_FULL_INT);

    return 1;  // Initialization successful
}

// FIFO clearing before data recording
uint8_t MAX30102_FIFO_Reset(void) {
	uint8_t part_id = 0;

	// Read MAX30102 ID to verify device presence
	if (MAX30102_ReadRegister(REG_PART_ID, &part_id) != HAL_OK || part_id != 0x15) {
		return 0;  // Device not found
	}

	// set the FIFO read and write pointers to 0, and reset overflow counter to 0
	MAX30102_WriteRegister(REG_FIFO_WR_PTR, 0x00);
	MAX30102_WriteRegister(REG_OVF_COUNTER, 0x00);
	MAX30102_WriteRegister(REG_FIFO_RD_PTR, 0x00);

	return 1;
}

/* USER CODE END 0 */

#endif /* INC_MAX30102_H_ */
