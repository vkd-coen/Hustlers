/*
 * motors.c
 *
 *  Created on: May 15, 2026
 *      Author: vikto
 */


#include "motors.h"
#include "tim.h"
#include "gpio.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Motors_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // FL
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // BL
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // FR
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // BR
    Motors_Stop();
}

static void set_direction(MotorId id, uint8_t forward) {
    switch (id) {
    case MOTOR_FL:
        HAL_GPIO_WritePin(M_FL_IN1_GPIO_Port, M_FL_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M_FL_IN2_GPIO_Port, M_FL_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MOTOR_FR:
        HAL_GPIO_WritePin(M_FR_IN1_GPIO_Port, M_FR_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M_FR_IN2_GPIO_Port, M_FR_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MOTOR_BL:
        HAL_GPIO_WritePin(M_BL_IN1_GPIO_Port, M_BL_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M_BL_IN2_GPIO_Port, M_BL_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case MOTOR_BR:
        HAL_GPIO_WritePin(M_BR_IN1_GPIO_Port, M_BR_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M_BR_IN2_GPIO_Port, M_BR_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    }
}

static void set_pwm(MotorId id, uint16_t duty) {
    // ARR = 999, so duty 0..999 = 0..100%
    switch (id) {
    case MOTOR_FL: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty); break;
    case MOTOR_BL: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty); break;
    case MOTOR_FR: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); break;
    case MOTOR_BR: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); break;
    }
}

void Motors_SetSpeed(MotorId id, int8_t speed_percent) {
    if (speed_percent > 100)  speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    set_direction(id, speed_percent >= 0);
    uint16_t duty = (uint16_t)((speed_percent >= 0 ? speed_percent : -speed_percent) * 10);
    set_pwm(id, duty);
}

void Motors_Drive(int8_t linear, int8_t turn) {
    int16_t left  = linear - turn;
    int16_t right = linear + turn;
    if (left > 100)  left = 100;   if (left < -100)  left = -100;
    if (right > 100) right = 100;  if (right < -100) right = -100;

    Motors_SetSpeed(MOTOR_FL, (int8_t)left);
    Motors_SetSpeed(MOTOR_BL, (int8_t)left);
    Motors_SetSpeed(MOTOR_FR, (int8_t)right);
    Motors_SetSpeed(MOTOR_BR, (int8_t)right);
}

void Motors_Stop(void) {
    Motors_SetSpeed(MOTOR_FL, 0);
    Motors_SetSpeed(MOTOR_FR, 0);
    Motors_SetSpeed(MOTOR_BL, 0);
    Motors_SetSpeed(MOTOR_BR, 0);
}
