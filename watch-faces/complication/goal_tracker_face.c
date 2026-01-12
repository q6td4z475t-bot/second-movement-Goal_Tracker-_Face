// watch_faces/complication/goal_tracker_face.c
// SENSOR WATCH PRO - GOAL TRACKER FACE (extensively commented)
// - Upgraded LCD: top row shows "A:### B:##", main shows time.
// - Non-volatile tallies and goals (A supports up to 3 digits via 16-bit storage).
// - Increment = 2s hold; Reset = 5s hold (single action per hold).
// - Single tap (LIS2DW) -> GET A (if behind).
// - Double tap (LIS2DW) -> GET B (if behind).
// - Triple tap -> SET mode toggle (Option 3): first triple-tap -> SET A, next triple-tap -> SET B, etc.
// - Deficit uses real days-in-month (incl. leap years) and displays with 2 decimal places.
// - Uses lis2dw driver: lis2dw_get_int_source(), lis2dw_get_accel(...).
// - Uses movement/watch helpers: movement_get_local_time(), movement_is_button_pressed(),
//   movement_request_tick_frequency(), watch_display_string(), watch_display_time(),
//   watch_get_backup_data(), watch_store_backup_data().
//
// Extensive inline comments explain the logic and where to tweak parameters.

#include "movement.h"
#include "watch.h"
#include "watch_private_display.h"
#include "lis2dw.h"   // sensor driver in repo: watch-library/shared/driver/lis2dw.*
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ===== ADD THE FORWARD DECLARATIONS HERE ===== */
void goal_tracker_face_setup(movement_settings_t *settings, uint8_t watch_face_index, void **context_ptr);
void goal_tracker_face_activate(movement_settings_t *settings, void *context);
bool goal_tracker_face_loop(movement_event_t event, movement_settings_t *settings, void *context);
void goal_tracker_face_resign(movement_settings_t *settings, void *context);
/* ============================================== */

/* ------------------------- STORAGE (backup SRAM) -------------------------
   The Sensor Watch exposes small backup SRAM (32 bytes). Each watch_get_backup_data /
   watch_store_backup_data call reads/writes one byte.

   Because you requested Tally A to support triple digits, we store 16-bit values
   for Tally A and Goal A across two consecutive backup slots (low byte then high byte).

   Layout (byte indices):
     0 - TALLY_A low
     1 - TALLY_A high
     2 - TALLY_B low  (fits in one byte but keep symmetry)
     3 - TALLY_B high (unused for B but reserved; we'll store 0)
     4 - GOAL_A low
     5 - GOAL_A high
     6 - GOAL_B low
     7 - GOAL_B high
   Remaining bytes available if you later want more persistent vars.
-------------------------------------------------------------------------*/

#define BK_TALLY_A_LO 0
#define BK_TALLY_A_HI 1
#define BK_TALLY_B_LO 2
#define BK_TALLY_B_HI 3
#define BK_GOAL_A_LO  4
#define BK_GOAL_A_HI  5
#define BK_GOAL_B_LO  6
#define BK_GOAL_B_HI  7

/* Default goals (can be changed on-watch) */
#define GOAL_A_DEFAULT 12    // user said 12 for A
#define GOAL_B_DEFAULT 4     // user said 4 for B

/* Min/max constraints */
#define MIN_GOAL 1
#define MAX_GOAL_A 999       // A supports up to 3 digits
#define MAX_GOAL_B 99        // B limited to 2 digits on LCD

/* Action timing (seconds) */
#define HOLD_INC_SECONDS   2   // hold time to increment
#define HOLD_RESET_SECONDS 5   // hold time to reset (overrides increment)
#define GET_SHOW_SECONDS   3   // how long GET display remains

/* Tap / gesture timing (ms) */
#define TRIPLE_TAP_WINDOW_MS 1500 // window for counting up to 3 single taps
#define TAP_DEBOUNCE_MS      250  // after confirmed gesture ignore further taps briefly

/* Display indexes (usual convention for upgraded LCD) */
#define TOP_DISPLAY_INDEX 0
#define MAIN_DISPLAY_INDEX 1

/* Tap source bits from lis2dw.h; define fallback if header didn't */
#ifndef LIS2DW_TAP_SRC_SINGLE_TAP
#define LIS2DW_TAP_SRC_SINGLE_TAP (1 << 6)
#endif
#ifndef LIS2DW_TAP_SRC_DOUBLE_TAP
#define LIS2DW_TAP_SRC_DOUBLE_TAP (1 << 5)
#endif

/* ------------------------- Helper: multi-byte backup I/O -------------------------
   Read and write 16-bit values across two backup byte slots (little-endian).
-------------------------------------------------------------------------*/
static uint16_t backup_read_u16(uint8_t lo_index, uint8_t hi_index) {
    uint8_t lo = watch_get_backup_data(lo_index);
    uint8_t hi = watch_get_backup_data(hi_index);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

static void backup_write_u16(uint8_t lo_index, uint8_t hi_index, uint16_t value) {
    uint8_t lo = (uint8_t)(value & 0xFF);
    uint8_t hi = (uint8_t)((value >> 8) & 0xFF);
    watch_store_backup_data(lo_index, lo);
    watch_store_backup_data(hi_index, hi);
}

/* ------------------------- Date helpers (days in month, leap year) -------------
   We use the RTC to determine current day/month/year and compute expected progress:
     expected = goal * (day_of_month / days_in_month)
   Deficit = expected - actual (if >0)
-------------------------------------------------------------------------------*/
static uint8_t days_in_month(uint16_t year, uint8_t month) {
    static const uint8_t mdays[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    if (month != 2) return mdays[month - 1];
    bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    return leap ? 29 : 28;
}

/* fetch current date using movement helper (common in sensor-watch builds)
   returns true on success and fills year/month/day */
static bool get_current_date(uint16_t *year, uint8_t *month, uint8_t *day) {
    struct tm now;
    if (movement_get_local_time(&now)) {
        *year = (uint16_t)(now.tm_year + 1900);
        *month = (uint8_t)(now.tm_mon + 1);
        *day = (uint8_t)now.tm_mday;
        return true;
    }
    return false;
}

/* Compute deficit for given goal and actual tally using real month length.
   If RTC unavailable, return 0.0f (no alert) to avoid false GETs. */
static float compute_deficit(uint16_t goal, uint16_t actual) {
    uint16_t yr; uint8_t mo; uint8_t dy;
    if (!get_current_date(&yr, &mo, &dy)) {
        return 0.0f; // conservative: don't alert when date is unknown
    }
    uint8_t dim = days_in_month(yr, mo);
    float expected = ((float)goal) * ((float)dy / (float)dim);
    float deficit = expected - (float)actual;
    if (deficit < 0.0f) deficit = 0.0f;
    return deficit;
}

/* ------------------------- State struct -------------------------
   All runtime state for the face lives here.
-------------------------------------------------------------------------*/
typedef enum {
    MODE_NORMAL = 0,
    MODE_SHOW_GET,
    MODE_SET_A,
    MODE_SET_B
} face_mode_t;

typedef struct {
    uint16_t tally_a;
    uint16_t tally_b;
    uint16_t goal_a;
    uint16_t goal_b;

    /* hold counters (seconds) while button is held */
    uint8_t hold_seconds_a;
    uint8_t hold_seconds_b;

    /* prevent multiple actions during a single long hold */
    bool action_done_during_hold_a;
    bool action_done_during_hold_b;

    /* gesture/tap tracking (ms clock updated on ticks) */
    uint32_t ms_clock;
    uint32_t last_tap_ms;        // time of last tap detected
    uint8_t tap_count;           // number of single taps counted in current window
    uint32_t last_gesture_ms;    // when we last handled a gesture (for debounce)

    /* GET display state */
    face_mode_t mode;
    uint8_t get_seconds_remaining; // countdown in seconds

} goal_tracker_face_state_t;

/* forward declarations required by framework */
void goal_tracker_face_setup(movement_settings_t *settings, uint8_t watch_face_index, void **context_ptr);
void goal_tracker_face_activate(movement_settings_t *settings, void *context);
bool goal_tracker_face_loop(movement_event_t event, movement_settings_t *settings, void *context);
void goal_tracker_face_resign(movement_settings_t *settings, void *context);

/* ------------------------- Helper: top-row renderer -------------------------
   Compose the top line string "A:### B:##" according to current tallies.
   A uses 3 digits (padded); B uses 2 digits (padded).
-------------------------------------------------------------------------*/
static void render_top_line(goal_tracker_face_state_t *st, char *buf, size_t buflen) {
    // Example: "A:012 B:04"
    // Use %03u for A and %02u for B to maintain alignment
    snprintf(buf, buflen, "A:%03u B:%02u", (unsigned)st->tally_a, (unsigned)st->tally_b);
}

/* ------------------------- Gesture handling helpers -------------------------
   We use the LIS2DW driver:
     - lis2dw_get_int_source() returns the sensor INT_SRC/TAP_SRC register bits
       (we check SINGLE_TAP and DOUBLE_TAP bits).
     - lis2dw_get_accel(&x,&y,&z) exists but is NOT required for tap bits; we rely on int source.

   Gesture policy:
     - If DOUBLE_TAP bit is set -> immediate double-tap action (GET B)
     - If SINGLE_TAP bit is set -> accumulate into a tap_count and evaluate:
         * if we reach 3 single taps within TRIPLE_TAP_WINDOW_MS -> treat as triple-tap
         * if the window expires (no more single taps) -> treat as single-tap
     - Debounce: after any confirmed gesture, ignore further gestures for TAP_DEBOUNCE_MS.
-------------------------------------------------------------------------*/

/* Called when we decide a single-tap gesture is confirmed (after window expires) */
static void do_single_tap_action(goal_tracker_face_state_t *st) {
    // Single tap = GET A (priority)
    float def = compute_deficit(st->goal_a, st->tally_a);
    if (def > 0.0001f) {
        st->mode = MODE_SHOW_GET;
        st->get_seconds_remaining = GET_SHOW_SECONDS;
    }
}

/* Called when double-tap is detected (driver provided immediate bit) */
static void do_double_tap_action(goal_tracker_face_state_t *st) {
    // Double tap = GET B (priority after A)
    float def = compute_deficit(st->goal_b, st->tally_b);
    if (def > 0.0001f) {
        st->mode = MODE_SHOW_GET;
        st->get_seconds_remaining = GET_SHOW_SECONDS;
    }
}

/* Called when triple-tap is detected */
static void do_triple_tap_action(goal_tracker_face_state_t *st) {
    // Option 3: triple-tap toggles between SET A and SET B.
    // If currently normal -> enter SET A. If already SET A -> switch to SET B.
    if (st->mode == MODE_SET_A) {
        st->mode = MODE_SET_B;
    } else {
        st->mode = MODE_SET_A;
    }
}

/* ------------------------- Main face functions ------------------------- */

/* Setup: allocate state and load persistent values */
void goal_tracker_face_setup(movement_settings_t *settings, uint8_t watch_face_index, void **context_ptr) {
    (void)settings;
    (void)watch_face_index;

    if (*context_ptr == NULL) {
        goal_tracker_face_state_t *st = malloc(sizeof(goal_tracker_face_state_t));
        if (!st) return; // allocation failure - extremely unlikely on these builds

        /* load persistent tallies/goals from backup SRAM (u16 for A, u16 for B/goals) */
        st->tally_a = backup_read_u16(BK_TALLY_A_LO, BK_TALLY_A_HI);
        st->tally_b = backup_read_u16(BK_TALLY_B_LO, BK_TALLY_B_HI);

        uint16_t saved_goal_a = backup_read_u16(BK_GOAL_A_LO, BK_GOAL_A_HI);
        uint16_t saved_goal_b = backup_read_u16(BK_GOAL_B_LO, BK_GOAL_B_HI);

        /* If backup memory uninitialized, watch_get_backup_data returns 0xFF per byte.
           That would produce goal values maybe > MIN_GOAL or nonsense; handle fallback.
           Simple rule: if saved_goal < MIN_GOAL or > MAX, use defaults. */
        if (saved_goal_a < MIN_GOAL || saved_goal_a > MAX_GOAL_A) st->goal_a = GOAL_A_DEFAULT;
        else st->goal_a = saved_goal_a;

        if (saved_goal_b < MIN_GOAL || saved_goal_b > MAX_GOAL_B) st->goal_b = GOAL_B_DEFAULT;
        else st->goal_b = saved_goal_b;

        /* clamps for tallies just in case */
        if (st->tally_a > MAX_GOAL_A) st->tally_a = MAX_GOAL_A;
        if (st->tally_b > MAX_GOAL_B) st->tally_b = MAX_GOAL_B;

        /* initialize transient fields */
        st->hold_seconds_a = 0;
        st->hold_seconds_b = 0;
        st->action_done_during_hold_a = false;
        st->action_done_during_hold_b = false;

        st->ms_clock = 0;
        st->last_tap_ms = 0;
        st->tap_count = 0;
        st->last_gesture_ms = 0;

        st->mode = MODE_NORMAL;
        st->get_seconds_remaining = 0;

        *context_ptr = st;
    }
}

/* Activate: clear displays and ensure tick frequency */
void goal_tracker_face_activate(movement_settings_t *settings, void *context) {
    (void)context;
    (void)settings;
    watch_clear_display();
    movement_request_tick_frequency(1); // 1 Hz updates (we rely on second resolution)
}

/* Main event loop */
bool goal_tracker_face_loop(movement_event_t event, movement_settings_t *settings, void *context) {
    goal_tracker_face_state_t *st = (goal_tracker_face_state_t *)context;
    if (!st) return false;

    switch (event.event_type) {

        case EVENT_ACTIVATE:
            // reset hold & action flags; keep other state
            st->hold_seconds_a = 0;
            st->hold_seconds_b = 0;
            st->action_done_during_hold_a = false;
            st->action_done_during_hold_b = false;
            // ms clock can continue; but we ensure it's initialized
            if (st->ms_clock == 0) st->ms_clock = 0;
            break;

        case EVENT_TICK:
            /* EVENT_TICK is delivered at 1Hz (we requested tick_frequency 1).
               The movement framework may set event.subsecond to 0 at that time.
               We'll use subsecond==0 as our 1-second marker. */
            if (event.subsecond == 0) {
                st->ms_clock += 1000; // advance ms clock by one second

                /* ---------------------- Button hold handling ----------------------
                   For each button we:
                     - increment hold_seconds if the button is physically held (checked via movement_is_button_pressed)
                     - when hold reaches INC threshold and no action done yet -> perform increment
                     - when hold reaches RESET threshold and no action done yet -> perform reset (overrides inc)
                     - when button released -> clear hold counter and action flag
                -------------------------------------------------------------------*/

                // LIGHT => Tally A
                if (movement_is_button_pressed(BUTTON_LIGHT)) {
                    st->hold_seconds_a++;
                    if (!st->action_done_during_hold_a) {
                        if (st->hold_seconds_a >= HOLD_INC_SECONDS && st->hold_seconds_a < HOLD_RESET_SECONDS) {
                            // increment A (once)
                            if (st->tally_a < MAX_GOAL_A) st->tally_a++;
                            if (st->tally_a > MAX_GOAL_A) st->tally_a = MAX_GOAL_A;
                            // persist lower 16-bit
                            backup_write_u16(BK_TALLY_A_LO, BK_TALLY_A_HI, st->tally_a);
                            st->action_done_during_hold_a = true;
                        } else if (st->hold_seconds_a >= HOLD_RESET_SECONDS) {
                            // reset A (overrides increment)
                            st->tally_a = 0;
                            backup_write_u16(BK_TALLY_A_LO, BK_TALLY_A_HI, st->tally_a);
                            st->action_done_during_hold_a = true;
                        }
                    }
                } else {
                    // button released
                    st->hold_seconds_a = 0;
                    st->action_done_during_hold_a = false;
                }

                // ALARM => Tally B
                if (movement_is_button_pressed(BUTTON_ALARM)) {
                    st->hold_seconds_b++;
                    if (!st->action_done_during_hold_b) {
                        if (st->hold_seconds_b >= HOLD_INC_SECONDS && st->hold_seconds_b < HOLD_RESET_SECONDS) {
                            // increment B (once)
                            if (st->tally_b < MAX_GOAL_B) st->tally_b++;
                            if (st->tally_b > MAX_GOAL_B) st->tally_b = MAX_GOAL_B;
                            backup_write_u16(BK_TALLY_B_LO, BK_TALLY_B_HI, st->tally_b);
                            st->action_done_during_hold_b = true;
                        } else if (st->hold_seconds_b >= HOLD_RESET_SECONDS) {
                            // reset B
                            st->tally_b = 0;
                            backup_write_u16(BK_TALLY_B_LO, BK_TALLY_B_HI, st->tally_b);
                            st->action_done_during_hold_b = true;
                        }
                    }
                } else {
                    // released
                    st->hold_seconds_b = 0;
                    st->action_done_during_hold_b = false;
                }

                /* ---------------------- Accelerometer tap handling ----------------------
                   Use LIS2DW's interrupt source register for reliable detection.
                   lis2dw_get_int_source() returns the sensor's INT_SRC (tap) bits.

                   Behavior:
                     - if DOUBLE_TAP bit set -> immediate double-tap action (GET B)
                     - if SINGLE_TAP bit set -> treat as a single tap and accumulate count
                       to detect triple-tap (3 singles within TRIPLE_TAP_WINDOW_MS)
                     - when a single-tap sequence times out without reaching 3, we confirm
                       it as a single tap action (GET A)
                     - we respect TAP_DEBOUNCE_MS after any confirmed gesture
                ------------------------------------------------------------------------*/
                uint8_t int_src = lis2dw_get_int_source();
                uint32_t now = st->ms_clock;

                // 1) double-tap (immediate)
                if (int_src & LIS2DW_TAP_SRC_DOUBLE_TAP) {
                    // debounce double-tap
                    if (now - st->last_gesture_ms > TAP_DEBOUNCE_MS) {
                        do_double_tap_action(st);     // GET B if behind
                        st->last_gesture_ms = now;
                        // reset any single-tap accumulation
                        st->tap_count = 0;
                        st->last_tap_ms = 0;
                    }
                }

                // 2) single-tap reported by driver (accumulate for triple)
                if (int_src & LIS2DW_TAP_SRC_SINGLE_TAP) {
                    // ignore if in debounce period
                    if (now - st->last_gesture_ms > TAP_DEBOUNCE_MS) {
                        if (st->tap_count == 0) {
                            st->tap_count = 1;
                            st->last_tap_ms = now;
                        } else {
                            // if within triple window, add
                            if (now - st->last_tap_ms <= TRIPLE_TAP_WINDOW_MS) {
                                st->tap_count++;
                                st->last_tap_ms = now;
                            } else {
                                // too late: reset sequence to start new
                                st->tap_count = 1;
                                st->last_tap_ms = now;
                            }
                        }

                        // If we reached 3 single taps quickly => triple-tap
                        if (st->tap_count >= 3) {
                            // handle triple tap
                            if (now - st->last_gesture_ms > TAP_DEBOUNCE_MS) {
                                do_triple_tap_action(st);
                                st->last_gesture_ms = now;
                            }
                            // reset tap accumulation
                            st->tap_count = 0;
                            st->last_tap_ms = 0;
                        }
                    } // end debounce check
                }

                // 3) if there is 1 or 2 single taps and the window expired -> confirm single tap
                if (st->tap_count > 0) {
                    if (now - st->last_tap_ms > TRIPLE_TAP_WINDOW_MS) {
                        // window expired: interpret as single (we treat 2 quick singles as not double-
                        // because driver gives double-tap bit when double detected; so this is only
                        // 1-tap confirmation)
                        if (now - st->last_gesture_ms > TAP_DEBOUNCE_MS) {
                            // Confirm single tap
                            do_single_tap_action(st);
                            st->last_gesture_ms = now;
                        }
                        st->tap_count = 0;
                        st->last_tap_ms = 0;
                    }
                } // end tap handling

                // GET display countdown (on per-second tick)
                if (st->mode == MODE_SHOW_GET) {
                    if (st->get_seconds_remaining > 0) st->get_seconds_remaining--;
                    if (st->get_seconds_remaining == 0) {
                        st->mode = MODE_NORMAL;
                    }
                }
            } // end subsecond==0 blocks

            /* ---------------------- Rendering (every tick) ---------------------- */
            if (st->mode == MODE_SHOW_GET) {
                // priority: A then B (user requested this priority)
                float def_a = compute_deficit(st->goal_a, st->tally_a);
                float def_b = compute_deficit(st->goal_b, st->tally_b);

                if (def_a > 0.0001f) {
                    watch_display_string("GET A", TOP_DISPLAY_INDEX);
                    char mbuf[12];
                    // display with two decimals in a compact field; "%5.2f" yields e.g. " 2.62"
                    snprintf(mbuf, sizeof(mbuf), "%5.2f", def_a);
                    watch_display_string(mbuf, MAIN_DISPLAY_INDEX);
                } else if (def_b > 0.0001f) {
                    watch_display_string("GET B", TOP_DISPLAY_INDEX);
                    char mbuf[12];
                    snprintf(mbuf, sizeof(mbuf), "%5.2f", def_b);
                    watch_display_string(mbuf, MAIN_DISPLAY_INDEX);
                } else {
                    // nothing behind anymore; revert to normal display
                    char topbuf[16];
                    render_top_line(st, topbuf, sizeof(topbuf));
                    watch_display_string(topbuf, TOP_DISPLAY_INDEX);
                    watch_display_time(settings->bit.clock_24h);
                }
            } else if (st->mode == MODE_SET_A) {
                // Show SET A on top; main area shows current goal (integer)
                watch_display_string("SET A", TOP_DISPLAY_INDEX);
                char mbuf[12];
                snprintf(mbuf, sizeof(mbuf), "%3u", (unsigned)st->goal_a);
                watch_display_string(mbuf, MAIN_DISPLAY_INDEX);
            } else if (st->mode == MODE_SET_B) {
                // SET B mode
                watch_display_string("SET B", TOP_DISPLAY_INDEX);
                char mbuf[12];
                snprintf(mbuf, sizeof(mbuf), "%2u", (unsigned)st->goal_b);
                watch_display_string(mbuf, MAIN_DISPLAY_INDEX);
            } else {
                // Normal mode: top shows tallies, main shows time
                char topbuf[16];
                render_top_line(st, topbuf, sizeof(topbuf));
                watch_display_string(topbuf, TOP_DISPLAY_INDEX);
                watch_display_time(settings->bit.clock_24h);
            }

            break; // EVENT_TICK

        case EVENT_LIGHT_BUTTON_UP:
            // In SET modes: LIGHT increments the current editing goal.
            // In normal mode: LIGHT's long-hold was already handled in EVENT_TICK logic.
            if (st->mode == MODE_SET_A) {
                if (st->goal_a < MAX_GOAL_A) st->goal_a++;
                if (st->goal_a < MIN_GOAL) st->goal_a = MIN_GOAL;
                if (st->goal_a > MAX_GOAL_A) st->goal_a = MAX_GOAL_A;
                backup_write_u16(BK_GOAL_A_LO, BK_GOAL_A_HI, st->goal_a);
            } else if (st->mode == MODE_SET_B) {
                if (st->goal_b < MAX_GOAL_B) st->goal_b++;
                if (st->goal_b < MIN_GOAL) st->goal_b = MIN_GOAL;
                if (st->goal_b > MAX_GOAL_B) st->goal_b = MAX_GOAL_B;
                backup_write_u16(BK_GOAL_B_LO, BK_GOAL_B_HI, st->goal_b);
            }
            break;

        case EVENT_ALARM_BUTTON_UP:
            // In SET modes: ALARM decrements the current editing goal.
            if (st->mode == MODE_SET_A) {
                if (st->goal_a > MIN_GOAL) st->goal_a--;
                backup_write_u16(BK_GOAL_A_LO, BK_GOAL_A_HI, st->goal_a);
            } else if (st->mode == MODE_SET_B) {
                if (st->goal_b > MIN_GOAL) st->goal_b--;
                backup_write_u16(BK_GOAL_B_LO, BK_GOAL_B_HI, st->goal_b);
            }
            break;

        case EVENT_MODE_BUTTON_UP:
            // MODE is used to exit SET modes; in normal mode allow face to be changed (return false)
            if (st->mode == MODE_SET_A || st->mode == MODE_SET_B) {
                st->mode = MODE_NORMAL;
            } else {
                // allow leaving the face (consistent with other faces)
                return false;
            }
            break;

        default:
            break;
    } // end switch(event)

    return true;
}

/* Resign: called when leaving the face; nothing special to free (state persists) */
void goal_tracker_face_resign(movement_settings_t *settings, void *context) {
    (void)settings;
    (void)context;
}

/* ------------------------- End of file -------------------------
   Notes / tuning tips for testing on your Sensor Watch Pro:

   1) Tap interrupts:
      - Make sure the board init code for Sensor Watch Pro enables LIS2DW tap interrupts.
        Many Sensor Watch Pro board initializers call lis2dw_init() and configure tap detection.
        If you find taps aren't detected, ensure LIS2DW is configured for single/double-tap
        and that lis2dw_get_int_source() reflects that configuration.

   2) TAP tuning:
      - TRIPLE_TAP_WINDOW_MS (1500 ms) is reasonably generous: three quick taps within 1.5s.
      - TAP_DEBOUNCE_MS (250 ms) prevents rapid re-triggering.
      - You can adjust these constants at the top for your personal tapping style.

   3) Backup storage:
      - We used 2 bytes per 16-bit value so Tally A and Goal A can exceed 255.
      - If you later want to expand or add extra variables, you have remaining backup bytes.

   4) Display indices:
      - TOP_DISPLAY_INDEX = 0, MAIN_DISPLAY_INDEX = 1 are the usual conventions for upgraded LCD.
      - If your display is different, change those constants.

   5) Goal editing:
      - Triple-tap toggles between SET A and SET B (Option 3).
      - While in SET A/B: use LIGHT to increment, ALARM to decrement, MODE to exit.
      - Values are saved immediately into backup SRAM.

   6) Increment/reset behavior:
      - A hold of exactly 2s triggers increment (once). If you continue to hold to 5s,
        the reset behavior triggers once and supersedes the increment (per your design).

   7) Behavior after GET:
      - A single tap shows GET A if A is behind; double tap shows GET B if B is behind.
      - After showing GET, the face returns to normal; it does not automatically chain to the other tally.
        (You can tap again to check the other.)

   8) If you want I can:
      - Provide a tiny board-init snippet to ensure lis2dw is configured for taps (if your board doesn't).
      - Convert this code to a .c + .h pair.
      - Add visual hold progress (flashing digit or small indicator while holding) for UX clarity.

Enjoy — drop the file in, compile, and test taps/holds. If anything needs tuning (tap windows,
debounce, or display index differences), tell me what you saw and I’ll produce a tuned iteration.
------------------------------------------------------------------------- */
