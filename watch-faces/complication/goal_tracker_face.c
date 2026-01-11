// movement/watch_faces/goal_tracker/goal_tracker_face.c
// WATCH_FACE_GOAL_TRACKER (simplified tally face)
// Counts A and B using long holds on LIGHT and ALARM. 

#include "movement.h"
#include "watch.h"
#include "goal_tracker_face.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

const watch_face_t goal_tracker_face = {
    goal_tracker_face_setup,
    goal_tracker_face_activate,
    goal_tracker_face_loop,
    goal_tracker_face_resign
};

/* -------- Backup SRAM layout (bytes) ----------
 * 0: TALLY_A_LO
 * 1: TALLY_A_HI
 * 2: TALLY_B_LO
 * 3: TALLY_B_HI
 * --------------------------------------------*/
#define BK_TALLY_A_LO 0
#define BK_TALLY_A_HI 1
#define BK_TALLY_B_LO 2
#define BK_TALLY_B_HI 3

/* Defaults and limits */
#define MAX_TALLY_A 999
#define MAX_TALLY_B 999

/* Hold timings in seconds */
#define INC_HOLD_SECONDS   2   // hold 2+ seconds to increment
#define RESET_HOLD_SECONDS 5   // hold 5+ seconds to reset

// Small helpers for 16-bit values in backup RAM
static uint16_t backup_read_u16(uint8_t lo_idx, uint8_t hi_idx) {
    uint8_t lo = watch_get_backup_data(lo_idx);
    uint8_t hi = watch_get_backup_data(hi_idx);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

static void backup_write_u16(uint8_t lo_idx, uint8_t hi_idx, uint16_t v) {
    watch_store_backup_data(lo_idx, (uint8_t)(v & 0xFF));
    watch_store_backup_data(hi_idx, (uint8_t)((v >> 8) & 0xFF));
}

/* ----------------- state ----------------- */

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

// We use a single static instance instead of dynamic allocation. 
static goal_tracker_state_t goal_tracker_state;

/* -------------- rendering helpers ------------- */

static void render_top_line(goal_tracker_state_t *s, char *buf, size_t len) {
    // A and B up to 3 digits each
    snprintf(buf, len, "A:%03u B:%03u",
             (unsigned)s->tally_a,
             (unsigned)s->tally_b);
}

/* -------------- Movement face API ------------- */

void goal_tracker_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void)watch_face_index;

    // Always point context at our static state struct. 
    goal_tracker_state_t *s = &goal_tracker_state;

    s->tally_a = backup_read_u16(BK_TALLY_A_LO, BK_TALLY_A_HI);
    s->tally_b = backup_read_u16(BK_TALLY_B_LO, BK_TALLY_B_HI);

    if (s->tally_a > MAX_TALLY_A) s->tally_a = 0;
    if (s->tally_b > MAX_TALLY_B) s->tally_b = 0;

    s->hold_sec_a = 0;
    s->hold_sec_b = 0;
    s->light_held = false;
    s->alarm_held = false;
    s->action_done_a = false;
    s->action_done_b = false;

    *context_ptr = s;
}

void goal_tracker_face_activate(void *context) {
    (void)context;
    watch_clear_display();
    movement_request_tick_frequency(1); // 1 Hz tick
}

bool goal_tracker_face_loop(movement_event_t event, void *context) {
    goal_tracker_state_t *s = (goal_tracker_state_t *)context;
    if (! s) return false;

    switch (event. event_type) {

        case EVENT_ACTIVATE:
            s->hold_sec_a = 0;
            s->hold_sec_b = 0;
            s->light_held = false;
            s->alarm_held = false;
            s->action_done_a = false;
            s->action_done_b = false;
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            s->light_held = true;
            s->hold_sec_a = 0;
            s->action_done_a = false;
            break;

        case EVENT_LIGHT_BUTTON_UP: 
            s->light_held = false;
            s->hold_sec_a = 0;
            s->action_done_a = false;
            break;

        case EVENT_ALARM_BUTTON_DOWN:
            s->alarm_held = true;
            s->hold_sec_b = 0;
            s->action_done_b = false;
            break;

        case EVENT_ALARM_BUTTON_UP:
            s->alarm_held = false;
            s->hold_sec_b = 0;
            s->action_done_b = false;
            break;

        case EVENT_TICK:
            // Only run once per second
            if (event.subsecond == 0) {

                /* --- handle holds on LIGHT (A) --- */
                if (s->light_held) {
                    s->hold_sec_a++;
                    if (! s->action_done_a) {
                        if (s->hold_sec_a >= INC_HOLD_SECONDS &&
                            s->hold_sec_a < RESET_HOLD_SECONDS) {
                            if (s->tally_a < MAX_TALLY_A) s->tally_a++;
                            backup_write_u16(BK_TALLY_A_LO, BK_TALLY_A_HI, s->tally_a);
                            s->action_done_a = true;
                        } else if (s->hold_sec_a >= RESET_HOLD_SECONDS) {
                            s->tally_a = 0;
                            backup_write_u16(BK_TALLY_A_LO, BK_TALLY_A_HI, s->tally_a);
                            s->action_done_a = true;
                        }
                    }
                }

                /* --- handle holds on ALARM (B) --- */
                if (s->alarm_held) {
                    s->hold_sec_b++;
                    if (!s->action_done_b) {
                        if (s->hold_sec_b >= INC_HOLD_SECONDS &&
                            s->hold_sec_b < RESET_HOLD_SECONDS) {
                            if (s->tally_b < MAX_TALLY_B) s->tally_b++;
                            backup_write_u16(BK_TALLY_B_LO, BK_TALLY_B_HI, s->tally_b);
                            s->action_done_b = true;
                        } else if (s->hold_sec_b >= RESET_HOLD_SECONDS) {
                            s->tally_b = 0;
                            backup_write_u16(BK_TALLY_B_LO, BK_TALLY_B_HI, s->tally_b);
                            s->action_done_b = true;
                        }
                    }
                }

                /* --- update display --- */
                char top[16];
                render_top_line(s, top, sizeof(top));
                watch_display_string(top, 0); // top row, time line left blank
            }
            break;

        case EVENT_MODE_BUTTON_UP:
            // Leave this face
            return false;

        default:
            break;
    }

    return true; // stay on this face
}

void goal_tracker_face_resign(void *context) {
    goal_tracker_state_t *s = (goal_tracker_state_t *)context;
    if (!s) return;

    // Ensure tallies are saved
    backup_write_u16(BK_TALLY_A_LO, BK_TALLY_A_HI, s->tally_a);
    backup_write_u16(BK_TALLY_B_LO, BK_TALLY_B_HI, s->tally_b);
}
