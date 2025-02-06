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

MAX30102 pulseOximeter = {0};
FIFO_LED_DATA fifoData = {0};

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};

float currentTemperature = 25.0;

float currentBPM = 0;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
float valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
uint32_t lastBeatThreshold = 0;

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;

uint8_t redLEDCurrent = 0;
float lastREDLedCurrentCheck = 0;
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
    redLEDCurrent = 30;
    MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
    MAX30102_setLedCurrent(IR_LED, 10);
    //MAX30102_WriteRegister(REG_LED1_PA, 0x24);  // LED1 (Red)
    //MAX30102_WriteRegister(REG_LED2_PA, 0x24);  // LED2 (Infrared)

    // interrupts
    MAX30102_WriteRegister(REG_FIFO_CONFIG, ((32 - MAX30102_SAMPLES_PER_BURST) << BIT_FIFO_A_FULL_VAL)); // max value is 15 min is 0
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


uint8_t MAX30102_setLedCurrent(uint8_t led, float currentLevel)
{
	uint8_t value = 0;
	uint8_t ledRegister = 0;

	switch(led){
	case RED_LED: 	ledRegister = REG_LED1_PA; break;
	case IR_LED:	ledRegister = REG_LED2_PA; break;
	}

	// slope derived from MAX30102 DataSheet
	value = (uint8_t)(5.0 * currentLevel);

	if( MAX30102_WriteRegister(ledRegister, value) != HAL_OK){
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
		rawData.redLedRaw = ((max30102_sensor_data[n+0] << 16)) | (max30102_sensor_data[n+1] << 8) | max30102_sensor_data[n+2];
		rawData.irLedRaw = 	((max30102_sensor_data[n+3] << 16)) | (max30102_sensor_data[n+4] << 8) | max30102_sensor_data[n+5];

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


bool detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {
    	uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;

        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;


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
      //MAX30102_setLedCurrent(IR_LED, IrLedCurrent);
    }
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
    {
      redLEDCurrent--;
      MAX30102_setLedCurrent(RED_LED, redLEDCurrent);
      //MAX30102_setLedCurrent(IR_LED, IrLedCurrent);
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
		/*uint32_t lastBeatThreshold*/ 0,
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

	if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;

		float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

		//This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
		currentSpO2Value = 110.0 - 18.0 * ratioRMS;
		result.SpO2 = currentSpO2Value;

		if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
		{
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
	result.lastBeatThreshold = lastBeatThreshold;
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
