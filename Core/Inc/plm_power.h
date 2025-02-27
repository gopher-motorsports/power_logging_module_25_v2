/*
 * plm_power.h
 *
 *  Created on: Mar 31, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_POWER_H_
#define INC_PLM_POWER_H_

#include "main.h"
#include "GopherCAN.h"

#define NUM_OF_CHANNELS 11
#define MIN_5V_VOLTAGE_V 2.0f
#define MIN_VBAT_VOLTAGE_V 2.0f

// cooling control stuff
#define WHEEL_SPEED_FAN_OFF_THRESH_mph 20.0f
#define TRUST_VALUE_TIME_DELTA_ms 200

typedef struct {
    FLOAT_CAN_STRUCT* parameter;
    GPIO_TypeDef* enable_switch_port;
    uint16_t enable_switch_pin;
    uint8_t invert;
    uint8_t enabled;
    float amp_max;
    float ampsec_max;
    float ampsec_sum;
    uint32_t trip_time;
    uint32_t reset_delay_ms;
    uint32_t last_update;
    uint8_t overcurrent_count;
    uint8_t external_GPIO_on;
    uint8_t external_GPIO_off;
    uint8_t max_overcurrent_count;
    U8_CAN_STRUCT* overcurrentparam;
    U8_CAN_STRUCT* overcurrentcountparam;
} PLM_POWER_CHANNEL;

//uint8_t overcurrent_count_exceeded_event = 0;

extern PLM_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS];

void plm_power_update_channel(PLM_POWER_CHANNEL* channel);
void plm_cooling_control(void);

#endif /* INC_PLM_POWER_H_ */
