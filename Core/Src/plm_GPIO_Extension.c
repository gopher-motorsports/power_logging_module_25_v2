/*
 * plm_GPIO_Extension.c
 *
 * Includes helper functions that allow easy interfacing with peripherals connected to the
 * GPIO entension chip. The connected peripherals and their corresponding pins are listed below. These
 * are gathered from the Altium schematic.
 *
 * PIN 0 = Status LED
 * PIN 1 = Storage LED
 * PIN 2 = Overcurrent LED
 * PIN 3 = 5 Volt #0 Enable
 * PIN 4 = 5 Volt #1 Enable
 * PIN 5 = 5 Volt #2 Enable
 * PIN 6 = 5 Volt #3 Enable
 *
 * Pins for power channels are defined in the PLM_POWER_CHANNEL structs in plm_power.c
 *  Created on: Mar 15, 2024
 *      Author: joshwashburn
 */

#include "plm_power.h"
#include "main.h"
#include "GPIO_interface.h"

extern I2C_HandleTypeDef hi2c2;
uint8_t current_external_GPIO = 0b00000000;
uint8_t pData[2];

void GPIO_init() {
	// configure all ports on the GPIO extender as outputs
	pData[0] = CONFIGURATION_REGISTER;
	pData[1] = 0b00000000;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData, 2, 50);
	// Default state of output ports on GPIO extender is ON. This will turn them off in the initialization phase.
	pData[0] = OUTPUT_PORT_REGISTER;
	pData[1] = 0b00000000;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData, 2, 50);
}

void GPIO_Extension_On(int value){
	pData[0] = OUTPUT_PORT_REGISTER;
	pData[1] = value|current_external_GPIO;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData , 2, 50); // if there is an error, check the timing value, formerly 0x64.
	current_external_GPIO = value|current_external_GPIO;
}

void GPIO_Extension_Off(int value){
	pData[0] = OUTPUT_PORT_REGISTER;
	pData[1] = value&current_external_GPIO;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData , 2, 50); // if there is an error, check the timing value, formerly 0x64.
	current_external_GPIO = value&current_external_GPIO;
}

void GPIO_extension_overcurrent_LED(int state) {
	pData[0] = OUTPUT_PORT_REGISTER;
	if (state == 0) {
		pData[1] = (current_external_GPIO&0b11111011);
		current_external_GPIO = current_external_GPIO&0b11111011;
	} else if (state == 1) {
		pData[1] = (current_external_GPIO|0b00000100);
		current_external_GPIO = current_external_GPIO|0b00000100;
	}
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData , 2, 50); // if there is an error, check the timing value.
}

void GPIO_Extension_toggle(int pin) {
	pData[0] = OUTPUT_PORT_REGISTER;
	if (pin == 0) {
		pData[1] = current_external_GPIO^0b00000001;
		current_external_GPIO = current_external_GPIO^0b00000001;
//		if ((current_external_GPIO&0b00000001) == 0b00000001) {
//			pData[1] = (current_external_GPIO&0b11111110);
//			current_external_GPIO = current_external_GPIO&0b11111110;
//		} else if ((current_external_GPIO&0b00000001) == 0b00000000) {
//			current_external_GPIO = current_external_GPIO|0b00000001;
//		}
	} else if (pin == 1) {
		pData[1] = current_external_GPIO^0b00000010;
		current_external_GPIO = current_external_GPIO^0b00000010;
//		pData[1] = 0b00000010;
//		if ((current_external_GPIO&0b00000010) == 0b00000010) {
//			pData[1] = (current_external_GPIO&0b11111101);
//			current_external_GPIO = current_external_GPIO&0b11111101;
//		} else if ((current_external_GPIO&0b00000010) == 0b00000000) {
//					current_external_GPIO = current_external_GPIO|0b00000001;
//				}
	}
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDRESS, pData, 2, 50); // if there is an error, check the timing value.
}
