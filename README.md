# Sampling Architecture
The simultaneous sampling of 2 microphones at 48Khz is achieved using separate ADCs on the STM32, along with DMAs to manage memory transfers and i2s data transmission. 

![image](https://github.com/user-attachments/assets/71b898d0-c221-4426-a51d-5b107d982f9b)

The I2S PLL is configured to match the clock rate of the main PLL. This allows the sample timer to match the output i2s sample rate exactly but prevents the i2S transmission frequency from being exactly 48kHz. In reality, it is about 47.8kHz. This is acceptable, as it allows the DMAs to function without interfering with one another. This is not a concern because the output transmission frequency can never exactly match the expected rate for a different device anyway, and the the STM will provide a master clock. 

To achieve oversampling, a timer is used to trigger both sampling ADCs simultaneously at a multiple of the sampling frequency (in this case 8x). ADC1 and ADC2 are configured in "dual regular simultaneous mode," so the separate ADCs trigger and transmit their data together. 

DMA1 puts each ADC sample in a buffer, and once enough samples have been collected to satisfy oversampling, A CPU interrupt is triggered. The CPU will then sum all samples to achieve greater bit accuracy and remove DC offsets to center the audio signal at 0. The resulting signed value is then stored in a separate buffer to be transmitted by the second DMA out of the i2s interface. DMA 1 then continues to store sampled data from the ADCs.

DMA2 continuously loops over a buffer containing 2 16-bit signed values, each containing the latest processed sample for the respective microphone. The DMA sends these out of the i2s interface, alternating samples from the 2 microphones. The word select pin signals which microphone the sample originated from. 

# Pins
## RPi pinout
![image](https://github.com/user-attachments/assets/b4338e52-ca08-416a-b3d4-5f39972fb640)
## I2S
used to transmit the audio data to the Raspberry Pi 4.

CK: PB10 - Pi pin 12

WS: PB12 - Pi pin 35

SD: PC3 - Pi pin 38

## UART
used to transmit the MAX30102 heart rate and sp02 levels to the Raspberry Pi 4.

TX: PC12 - Pi pin 28

RX: (not needed) PD2

## Microphone Analog Inputs
PA0 and PA1

## MAX30102 Sensor

I2C SCL: PB6

I2S SDA: PB9

FIFO External Interrupt: PB0





