/*
 * behavior.c
 *
 *  Created on: May 15, 2026
 *      Author: vikto
 */

#include "behavior.h"
#include "motors.h"
#include <math.h>

/* These globals are owned by main.c — declared extern here */
extern uint8_t opponent_detected;
extern float   opponent_angle;
extern float   opponent_distance;

static BehaviorState state = BEHAVIOR_STOPPED;

void Behavior_Start(void) { state = BEHAVIOR_STARTUP; }
void Behavior_Stop(void)  { state = BEHAVIOR_STOPPED; Motors_Stop(); }

void Behavior_Tick(void) {
    switch (state) {

    case BEHAVIOR_STOPPED:
        Motors_Stop();
        break;

    case BEHAVIOR_STARTUP:
        // 5-second sumo startup delay (will be replaced by HIB03 trigger later)
        Motors_Stop();
        // Transition handled externally by start signal
        break;

    case BEHAVIOR_SEARCH:
        Motors_Drive(0, 35);  // Spin in place looking for opponent
        if (opponent_detected && opponent_distance < 1500.0f) {
            state = BEHAVIOR_ATTACK;
        }
        break;

    case BEHAVIOR_ATTACK: {
        if (!opponent_detected) {
            state = BEHAVIOR_SEARCH;
            break;
        }
        // Normalize angle to -180..180 (0 = straight ahead)
        float a = opponent_angle;
        if (a > 180.0f) a -= 360.0f;

        // Turn proportional to bearing, charge if pointed forward
        int8_t turn   = (int8_t)(a * 0.5f);
        if (turn > 60)  turn = 60;
        if (turn < -60) turn = -60;
        int8_t linear = (fabsf(a) < 15.0f) ? 100 : 30;

        Motors_Drive(linear, turn);
        break;
    }

    case BEHAVIOR_EDGE_ESCAPE:
        // Reverse for ~500 ms then re-search (handled with timer in main)
        Motors_Drive(-80, 0);
        break;
    }
}
