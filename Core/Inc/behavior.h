/*
 * behavior.h
 *
 *  Created on: May 15, 2026
 *      Author: vikto
 */

#ifndef INC_BEHAVIOR_H_
#define INC_BEHAVIOR_H_

#include <stdint.h>

typedef enum {
    BEHAVIOR_STARTUP,
    BEHAVIOR_SEARCH,
    BEHAVIOR_ATTACK,
    BEHAVIOR_EDGE_ESCAPE,
    BEHAVIOR_STOPPED
} BehaviorState;

/* Called every 50ms from main loop */
void Behavior_Tick(void);
void Behavior_Start(void);
void Behavior_Stop(void);

#endif /* INC_BEHAVIOR_H_ */
