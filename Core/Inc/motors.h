/*
 * motors.h
 *
 *  Created on: May 15, 2026
 *      Author: vikto
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"

typedef enum {
    MOTOR_FL,
    MOTOR_FR,
    MOTOR_BL,
    MOTOR_BR
} MotorId;

void Motors_Init(void);

/* Set one motor's speed: -100 = full reverse, 0 = stop, +100 = full forward */
void Motors_SetSpeed(MotorId id, int8_t speed_percent);

/* Differential drive helpers */
void Motors_Drive(int8_t linear_percent, int8_t turn_percent);
void Motors_Stop(void);

#endif /* INC_MOTORS_H_ */
