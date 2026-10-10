/* Header-only full-run extension: add this file beside main.h when flashing.
 * Existing $CMD/$STATUS operation is retained until a $FULL session handshake.
 * No HAL dependency: the exact button/gating logic can be tested on a host.
 */
#ifndef FULL_RUN_CONTROL_H
#define FULL_RUN_CONTROL_H
#include <stdint.h>

typedef struct {
    uint32_t session;
    uint32_t up_count, down_count, stop_count;
    uint32_t down_since;
    uint8_t active, base_rpm, instant_stop, fault;
    uint8_t down_tracking, long_sent;
} FullRunControl;

static inline void FullRun_Begin(volatile FullRunControl *s, uint32_t session)
{
    if (s->active && s->session == session) return; /* retransmit is idempotent */
    s->active = 1;
    s->session = session;
    s->base_rpm = 0;
    s->up_count = s->down_count = s->stop_count = 0;
    s->instant_stop = s->fault = s->down_tracking = s->long_sent = 0;
}

static inline void FullRun_InstantStop(volatile FullRunControl *s)
{
    if (!s->instant_stop) s->stop_count++;
    s->instant_stop = 1; /* preserve selected speed for next UP */
}

static inline void FullRun_ButtonTick(volatile FullRunControl *s, uint32_t now,
                                      uint8_t up_event, uint8_t down_event,
                                      uint8_t down_held)
{
    if (!s->active) return;
    if (down_held) {
        if (!s->down_tracking) {
            s->down_since = now;
            s->down_tracking = 1;
            s->long_sent = 0;
        }
        if (!s->long_sent && (uint32_t)(now - s->down_since) >= 800U) {
            FullRun_InstantStop(s);
            s->long_sent = 1;
        }
    } else {
        s->down_tracking = s->long_sent = 0;
    }
    /* Down takes precedence for simultaneous inputs. A long hold does not
     * decrement the saved speed: the short DOWN event is committed at release
     * by main.c, using the interrupt flag as debounced press evidence. */
    if (down_event && !s->instant_stop) {
        s->down_count++;
        s->base_rpm = s->base_rpm >= 5 ? (uint8_t)(s->base_rpm - 5) : 0;
    }
    if (up_event && !down_event && !down_held && !s->fault) {
        s->up_count++;
        if (s->instant_stop) {
            s->instant_stop = 0;
            if (s->base_rpm == 0) s->base_rpm = 5;
        } else if (s->base_rpm < 25) {
            s->base_rpm += 5;
        }
    }
}

static inline uint8_t FullRun_Allowed(const volatile FullRunControl *s)
{
    return !s->active || (s->base_rpm > 0 && !s->instant_stop && !s->fault);
}
#endif
