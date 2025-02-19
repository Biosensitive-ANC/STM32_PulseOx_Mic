/*
 * max30102.c
 *
 *  Created on: Feb 4, 2025
 *      Author: Robert
 */

#include "max30102.h"
#include "system.h"
#include "filter.h"
#include "math.h"
#include "stdlib.h"

const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
              3, 2, 1 } ;

MAX30102 pulseOximeter = {0};
FIFO_LED_DATA fifoData = {0};

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};

float currentTemperature = 25.0;

float currentBPM = 0;
int valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
float valuesBPMSum = 0;
uint8_t bpmIndex = 0;

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;

uint8_t redLEDCurrent = 0;
long lastREDLedCurrentCheck = 0;
PULSE_STATE_MACHINE currentPulseDetectorState = PULSE_IDLE;

uint8_t max30102_sensor_data[6 * MAX30102_SAMPLES_PER_BURST] = {0};


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
    MAX30102_WriteRegister(REG_MODE_CONFIG, (1 << 6));  // Reset MAX30102  (bit 6)
    HAL_Delay(10);

    // Configure SPO2 mode
    MAX30102_WriteRegister(REG_MODE_CONFIG, 0b011 << BIT_MODE);  // Use heart rate mode only (or 0x07 to enable SPO2)
    uint8_t spo2_bits = (0b01 << BIT_SPO2_ADC_RGE) |  (0b000 << BIT_SPO2_SR) | (0b01 << BIT_LED_PW); //pw of 11 browns out supply
    MAX30102_WriteRegister(REG_SPO2_CONFIG, spo2_bits);  // Set ADC range and sampling rate

    // Set LED brightness (0x24 represents medium brightness, adjustable)
    redLEDCurrent = 50;
    MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
    MAX30102_setLedCurrent(IR_LED, redLEDCurrent);

    // interrupts
    MAX30102_WriteRegister(REG_FIFO_CONFIG, (0b000 << BIT_SMP_AVG) | ((32 - MAX30102_SAMPLES_PER_BURST) << BIT_FIFO_A_FULL_VAL)); // max value is 15 min is 0
    MAX30102_WriteRegister(REG_INT1_EN, 1 << BIT_EN_A_FULL_INT);

    MAX30102_ClearInterrupt();

    return 1;  // Initialization successful
}

void MAX30102_ClearInterrupt(void) {
	uint8_t reset = 0;
	MAX30102_ReadRegister(REG_STATUS, &reset); // clears initial interrupt
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

	MAX30102_ClearInterrupt();

	return 1;
}


uint8_t MAX30102_setLedCurrent(uint8_t led, uint8_t currentLevel)
{
	uint8_t ledRegister = 0;

	switch(led){
	case RED_LED: 	ledRegister = REG_LED1_PA; break;
	case IR_LED:	ledRegister = REG_LED2_PA; break;
	}

	if( MAX30102_WriteRegister(ledRegister, currentLevel) != HAL_OK){
		return 0;
	}
	return 1;
}

uint8_t MAX30102_DumpFifo() {
	uint8_t retval = MAX30102_ReadFIFO(max30102_sensor_data, 6 * MAX30102_SAMPLES_PER_BURST);
	MAX30102_ClearInterrupt();
	return retval;
}

void MAX30102_ProcessData() {
	for (int i = 0 ; i < MAX30102_SAMPLES_PER_BURST; i++) {
		// heart rate mode only needs red, so only save red data initially long term
		FIFO_LED_DATA rawData = {0};
		uint8_t n = i*6;
		rawData.redLedRaw = ((max30102_sensor_data[n+0] << 16) | (max30102_sensor_data[n+1] << 8) | max30102_sensor_data[n+2]);
		rawData.irLedRaw = 	((max30102_sensor_data[n+3] << 16) | (max30102_sensor_data[n+4] << 8) | max30102_sensor_data[n+5]);

		pulseOximeter = pulseOximeter_update(rawData);

	}
}

void MAX30102_readTemperature(void)
{
	uint8_t tempNotDone = true;
	uint8_t readResult;
	int8_t tempFraction;
	uint8_t tempInteger;
	float temperature;

	// Initiate a temperature conversion
	MAX30102_WriteRegister(REG_TEMP_CONFIG, 1);

	// Wait for conversion finish
	while( tempNotDone != false )
	{ MAX30102_ReadRegister(REG_TEMP_CONFIG, &tempNotDone);	}

	// Read Die temperature integer register
	MAX30102_ReadRegister(REG_TEMP_INTEGER, &readResult);
	tempInteger = readResult;

	// Read Die temperature fraction register
	MAX30102_ReadRegister(REG_TEMP_FRACTION, &readResult);
	tempFraction = readResult;

	// Conversion factor found in MAX30102 DataSheet
	temperature = tempInteger + (tempFraction*0.0625);

	currentTemperature = temperature;
}

// Comparison function for qsort
int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// Function to calculate the median
double getMedian(int arr[], int size) {
    // Sort the array
    qsort(arr, size, sizeof(int), compare);

    return arr[size - 1 - 3];
}


bool detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint16_t currentBeatIndex = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    values_went_down = 0;
    return false;
  }

  currentBeatIndex++;

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if (!(sensor_value > prev_sensor_value))
      {
    	if (currentBeatIndex > 0) valuesBPM[bpmIndex] = currentBeatIndex;
    	currentBeatIndex = 0;

        bpmIndex++;

        if (bpmIndex >= PULSE_BPM_SAMPLE_SIZE){
        	bpmIndex = 0;
        	uint16_t medianIdx = getMedian(valuesBPM, (int)PULSE_BPM_SAMPLE_SIZE);

        	if(medianIdx > 0)
        		currentBPM = (60.0 * MAX30102_SAMPLE_RATE) / medianIdx;
        }

        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void balanceIntesities( float redLedDC, float IRLedDC )
{
	uint32_t currentTime = millis();
  if( currentTime - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
  {
	if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < 51)
    {
		redLEDCurrent++;
		MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
    }
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
    {
    	redLEDCurrent--;
    	MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
    }

    lastREDLedCurrentCheck = millis();
  }
}

MAX30102 pulseOximeter_update(FIFO_LED_DATA m_fifoData)
	{
		MAX30102 result = {
		/*bool pulseDetected*/ false,
		/*float heartBPM*/ 0.0,
		/*float irCardiogram*/ 0.0,
		/*float irDcValue*/ 0.0,
		/*float redDcValue*/ 0.0,
		/*float SaO2*/ currentSpO2Value,
		/*float dcFilteredIR*/ 0.0,
		/*float dcFilteredRed*/ 0.0,
		/*float temperature;*/ currentTemperature
	};

	dcFilterIR = dcRemoval( (float)m_fifoData.irLedRaw, dcFilterIR.w, ALPHA );
	dcFilterRed = dcRemoval( (float)m_fifoData.redLedRaw, dcFilterRed.w, ALPHA );

	float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
	lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

	irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
	redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
	samplesRecorded++;

	if( detectPulse( lpbFilterIR.result ) &&  samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;

		if( (pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0) )
		{
			float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );
			currentSpO2Value = 114.0 - 18.0 * ratioRMS;

			//uint8_t ratio =  ((redACValueSqSum/samplesRecorded) * 100) / (irACValueSqSum/samplesRecorded);
			//if (ratio > 183) ratio = 183;
			//currentSpO2Value = uch_spo2_table[ratio];

			result.SpO2 = currentSpO2Value;

			irACValueSqSum = 0;
			redACValueSqSum = 0;
			samplesRecorded = 0;
		}
	}

	balanceIntesities( dcFilterRed.w, dcFilterIR.w );

	result.heartBPM = currentBPM;
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = dcFilterIR.w;
	result.redDcValue = dcFilterRed.w;
	result.dcFilteredIR = dcFilterIR.result;
	result.dcFilteredRed = dcFilterRed.result;

	return result;
}

float MAX30102_getBPM(void)
{
	return pulseOximeter.heartBPM;
}
float MAX30102_getSPO2(void)
{
	return pulseOximeter.SpO2;
}
/* USER CODE END 0 */
