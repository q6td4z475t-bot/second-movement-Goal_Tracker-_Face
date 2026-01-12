#ifndef GOAL_TRACKER_FACE_H
#define GOAL_TRACKER_FACE_H

#include "movement.h"

typedef struct {
    uint16_t goals_completed;
    uint16_t goals_failed;
    uint16_t current_streak;
    uint16_t longest_streak;
    uint32_t last_reset_timestamp;
} goal_tracker_state_t;

extern const watch_face_t goal_tracker_face;

#endif // GOAL_TRACKER_FACE_H
