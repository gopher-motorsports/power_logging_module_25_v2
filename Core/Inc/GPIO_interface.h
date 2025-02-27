/*
 * GPIO_interface.h
 *
 *  Created on: Mar 14, 2024
 *      Author: stone
 */


#define DEVICE_ADDRESS 0b01000000
#define OUTPUT_PORT_REGISTER 0b00000001
#define CONFIGURATION_REGISTER 0b00000011
#define POLARITY_INVERSION_REGISTER 0b00000010

void GPIO_init(void);
void GPIO_Extension_On(int value);
void GPIO_extension_overcurrent_LED(int state);
void GPIO_Extension_toggle(int pin);
void GPIO_Extension_Off(int value);

