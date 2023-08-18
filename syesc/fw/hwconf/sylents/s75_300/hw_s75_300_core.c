/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include <math.h>
#include "mc_interface.h"
#include "syled.h"
#include <string.h>


// Threads
THD_FUNCTION(display_thread, arg);
static THD_WORKING_AREA(display_thread_wa, 256);
static bool display_thread_running = false;
static bool swi2c_running = false;

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

		// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	CURRENT_FILTER_OFF();

	// AUX pin
	AUX_OFF();
	palSetPadMode(AUX_GPIO, AUX_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);

			// Setup swi2c here
	if (!display_thread_running) {
		chThdCreateStatic(display_thread_wa, sizeof(display_thread_wa), NORMALPRIO, display_thread, NULL);
		display_thread_running = true;
	}

}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

float hw75_300_get_temp(void) {
	float t1 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float t2 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float t3 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float res = 0.0;

	if (t1 > t2 && t1 > t3) {
		res = t1;
	} else if (t2 > t1 && t2 > t3) {
		res = t2;
	} else {
		res = t3;
	}

	return res;
}


void sw_init_i2c(void) {

	palSetPadMode(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN,
			PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN,
			PAL_MODE_OUTPUT_PUSHPULL);

	palSetPad(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN);
	palSetPad(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN);

	swi2c_running = true;
}


void sw_start_i2c(void) {
	palSetPad(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN);
	palClearPad(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN);
	swi2c_running = true;
}

void sw_stop_i2c(void) {
	palSetPad(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN);
	palSetPad(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN);
	swi2c_running = false;
}

#define HIGH 1
#define LOW 0

#define setSCLK()    palSetPad(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN)
#define clearSCLK()  palClearPad(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN)
#define setSDIO()    palSetPad(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN)
#define clearSDIO()  palClearPad(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN)
#define SCLKSPEED	 25

void swi2cMasterTransmitByteNoStop(uint8_t data){
    uint8_t i;

    // Bit banging the data byte, LSB first
    for (i = 0; i < 8; i++) {
        // Ensure CLK is low before changing data
        clearSCLK();
        chThdSleepMicroseconds(SCLKSPEED/10);

        if (data & 0x01) {
            setSDIO();
        } else {
            clearSDIO();
        }
        // Shift to next bit
        data >>= 1;
        chThdSleepMicroseconds(SCLKSPEED);

        // After setting data, set CLK high to send
        setSCLK();
        chThdSleepMicroseconds(SCLKSPEED);
    }
}


void swi2cMasterTransmitByte(uint8_t data){
    // Start condition: when CLK is high, the DIN becomes low from high
    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    // Transmit data
    swi2cMasterTransmitByteNoStop(data);

    // End condition: when CLK is high, the DIN becomes high from low
    clearSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    setSDIO();
}


int swi2cMasterTransmitBytes(uint8_t addr, uint8_t numofbytes, uint8_t values[]){
    uint8_t i;

    // Start condition: when CLK is high, the DIN becomes low from high
    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    // Transmit address/command
    swi2cMasterTransmitByteNoStop(TM1640_ADDR_COMMAND | addr);

    // Transmit data bytes
    for (i = 0; i < numofbytes; i++) {
        swi2cMasterTransmitByteNoStop(values[i]);
    }

    // End condition: when CLK is high, the DIN becomes high from low
    clearSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    setSDIO();

    return 0;
}


int swi2cMasterLedDigitsUpper(uint32_t digits){
    uint8_t i;
	uint8_t txbuf[4];
	memset(txbuf, 0, 4 * sizeof(uint8_t));

	uint8_t numofbytes = intToDigits(digits, txbuf);
	// txbuf now holds the digits in the correct order
	// set the upper digits to zero
	for (i = numofbytes; i < 4; i++) {
		txbuf[i] = 0;
	}


    // Start condition: when CLK is high, the DIN becomes low from high
    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    // Transmit address/command
    swi2cMasterTransmitByteNoStop(TM1640_ADDR_COMMAND | ULED_MSB);

	if (digits == 0)
	{
		txbuf[0] = C7_0;
		numofbytes = 1;
	}
    // Transmit data bytes
    for (i = 0; i < 4; i++) {
        if (i < (4-numofbytes)) 
			swi2cMasterTransmitByteNoStop(0);
		else	
        	swi2cMasterTransmitByteNoStop(txbuf[i-(4-numofbytes)]);
    }

    // End condition: when CLK is high, the DIN becomes high from low
    clearSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    setSDIO();

    return 0;
}


int swi2cMasterLedDigitsLower(uint32_t digits){
    uint8_t i;
	uint8_t txbuf[3];
	memset(txbuf, 0, 3 * sizeof(uint8_t));

	uint8_t numofbytes = intToDigits(digits, txbuf);
	// txbuf now holds the digits in the correct order
	// set the upper digits to zero
	for (i = numofbytes; i < 3; i++) {
		txbuf[i] = 0;
	}

    // Start condition: when CLK is high, the DIN becomes low from high
    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    // Transmit address/command
    swi2cMasterTransmitByteNoStop(TM1640_ADDR_COMMAND | LLED_MSB);

	if (digits == 0)
	{
		txbuf[0] = C7_0;
		numofbytes = 1;
	}

    // Transmit data bytes
    for (i = 0; i < 3; i++) {
        if (i < (3-numofbytes)) 
			swi2cMasterTransmitByteNoStop(0);
		else	
        	swi2cMasterTransmitByteNoStop(txbuf[i-(3-numofbytes)]);
    }

    // End condition: when CLK is high, the DIN becomes high from low
    clearSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    clearSDIO();
    chThdSleepMicroseconds(SCLKSPEED);

    setSCLK();
    chThdSleepMicroseconds(SCLKSPEED/10);
    setSDIO();

    return 0;
}

int swi2cMasterLedBattLevel(uint32_t level){
	uint8_t txbuf[2];
	
    if (level > 0 && level <= 20) {
        txbuf[0] = SLED_BATT_20;
    } else if (level > 20 && level <= 40) {
        txbuf[0] = SLED_BATT_40;
    } else if (level > 40 && level <= 60) {
        txbuf[0] = SLED_BATT_60;
    } else if (level > 60 && level <= 80) {
        txbuf[0] = SLED_BATT_80;
    } else if (level > 80 && level <= 100) {
        txbuf[0] = SLED_BATT_100;
    } else {
        txbuf[0] = SLED_BATT_0;
    }

	txbuf[0] |= (SLED_BATT_HULL & 0xFF);	// mask in
	txbuf[1] = 0 | ((SLED_BATT_HULL>>8) & 0xFF);	// mask in 
	swi2cMasterTransmitBytes(BATLED_MSB, 2, txbuf);

	return 0;
}


void sw_init_i2cdisplay(void) {
    uint8_t txbuf[SIZE_MATRIX_COL];
	// Initialize txbuf with all 0s
	memset(txbuf, 0, SIZE_MATRIX_COL * sizeof(uint8_t));

    // Set Data Addr Increase Mode
    swi2cMasterTransmitByte(TM1640_DATA_COMMAND | TM1640_DATA_CINC);
    // Turn Display On
    swi2cMasterTransmitByte(TM1640_DISP_COMMAND 
						  | TM1640_DISP_CON 
						  | TM1640_DISP_BRIGHTNESS_8);
	
	// Clear Display
	swi2cMasterTransmitBytes(TM1640_DATA_COMMAND, SIZE_MATRIX_COL, txbuf);

}


THD_FUNCTION(display_thread, arg) {
    (void)arg;

    chRegSetThreadName("SW_I2C Display");

	float voltage = 0;
	float watt = 0;
	float current = 0;
	float level = 0;
	float duty = 0;
	uint32_t prevVoltageInt = 0;
	uint32_t prevWattInt = 0;
	uint32_t prevDutyInt = 0;
	uint32_t voltageInt = 0;
    uint32_t wattInt = 0;
    uint32_t dutyInt = 0;
    uint32_t currentInt = 0;
	uint8_t txbuf[1];

    volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();

    sw_init_i2c();
    chThdSleepMilliseconds(10);
	sw_init_i2cdisplay();    // Set Data Addr Increase Mode
    chThdSleepMilliseconds(10);
	txbuf[0] = SLED_DTYPE_W;// |
			//SLED_DTYPE_KMH | 
			//SLED_DTYPE_PERCENT |  
			//SLED_DTYPE_VOLT;
	swi2cMasterTransmitBytes(SLED_DTYPE_COLUMN, 1, txbuf);

	swi2cMasterLedBattLevel( 75 );
	swi2cMasterLedDigitsUpper( 35 );
	swi2cMasterLedDigitsLower( 22 );

for (;;) {

}
}
