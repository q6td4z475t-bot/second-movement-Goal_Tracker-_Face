#ifndef GOAL_TRACKER_FACE_H
#define GOAL_TRACKER_FACE_H

#include "movement. h"

#ifdef __cplusplus
extern "C" {
#endif

extern const watch_face_t goal_tracker_face;

void goal_tracker_face_setup(uint8_t watch_face_index, void **context_ptr);
void goal_tracker_face_activate(void *context);
bool goal_tracker_face_loop(movement_event_t event, void *context);
void goal_tracker_face_resign(void *context);

#ifdef __cplusplus
}
#endif

#endif // GOAL_TRACKER_FACE_H
