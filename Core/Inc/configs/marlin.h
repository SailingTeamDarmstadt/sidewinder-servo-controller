/*
 * marlin.h
 *
 *  Created on: Aug 24, 2024
 *  Author: malte
 *
 * Servo configuration file for Marlin/Sidewinder
 *
 */

#ifndef INC_CONFIGS_MARLIN_H_
#define INC_CONFIGS_MARLIN_H_

#ifdef SERVO_CONFIG
#error "More than one config enabled in main.c!"
#endif
#define SERVO_CONFIG

/* PWM inputs, depending on clock and prescaler */
const uint32_t PWM_MIN_PULSE = 1000;
const uint32_t PWM_MAX_PULSE = 2000;
/* PWM value > this is read as "switch on" */
const uint32_t SWITCH_ON_THRESHOLD = 1500;
/* Upper limit to check against broken connections */
const uint32_t SWITCH_ON_UPPER = 2000;
/* rudder channel limits */
const uint32_t CHAN1_OUT_MINVAL = 1100;
const uint32_t CHAN1_OUT_MAXVAL = 1900;
/* sail channel limits */
const uint32_t CHAN2_OUT_MINVAL = 1600;
const uint32_t CHAN2_OUT_MAXVAL = 1900;
/* main sail winch address used here */
const uint32_t CAN_ADDRESS_SAIL = 0xC0000;
const int32_t MAX_SAIL_ANGLE = 2147483647;
const int32_t MIN_SAIL_ANGLE = 0;
/* rudder can address */
const uint32_t CAN_ADDRESS_RUDDER = 0xB0000;
const int32_t MAX_RUDDER_ANGLE = 2147483647;
const int32_t MIN_RUDDER_ANGLE = -2147483647;

#endif /* INC_CONFIGS_MARLIN_H_ */
