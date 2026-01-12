#ifndef GOAL_TRACKER_FACE_H_
#define GOAL_TRACKER_FACE_H_

#include "movement. h"

typedef struct {
    uint16_t tally_a;
    uint16_t tally_b;
    uint8_t hold_sec_a;
    uint8_t hold_sec_b;
    bool light_held;
    bool alarm_held;
    bool action_done_a;
    bool action_done_b;
} goal_tracker_state_t;

void goal_tracker_face_setup(uint8_t watch_face_index, void **context_ptr);
void goal_tracker_face_activate(void *context);
bool goal_tracker_face_loop(movement_event_t event, void *context);
void goal_tracker_face_resign(void *context);

#define goal_tracker_face ((const watch_face_t){ \
    goal_tracker_face_setup, \
    goal_tracker_face_activate, \
    goal_tracker_face_loop, \
    goal_tracker_face_resign, \
    NULL \
})

#endif // GOAL_TRACKER_FACE_H_
EOF
