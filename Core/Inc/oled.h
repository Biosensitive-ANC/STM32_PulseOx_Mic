#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx_hal.h"

// OLED I2C address (default SSD1306 I2C address is 0x3C shifted left by 1)
#define OLED_ADDRESS 0x78

// OLED modes
#define OLED_CMD  0x00  // Control byte: command mode
#define OLED_DATA 0x40  // Control byte: data mode

// Function declarations
void OLED_Init(void);     // Initialize OLED
void OLED_Clear(void);    // Clear OLED display
void OLED_Fill(void);     // Light up all pixels on the OLED
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, char* str);

// Character dot matrix font table (ASCII 32-127, 8x8 font)
extern const uint8_t Font8x8[96][8];

#endif // __OLED_H
