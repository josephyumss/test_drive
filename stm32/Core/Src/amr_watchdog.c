#include "amr_watchdog.h"

#include <stddef.h>

void AmrWatchdog_Init(AmrCommandWatchdog *watchdog, uint32_t timeout_ms)
{
    if (watchdog != NULL) {
        watchdog->timeout_ms = timeout_ms;
        watchdog->last_valid_command_ms = 0U;
        watchdog->armed = false;
    }
}

void AmrWatchdog_Kick(AmrCommandWatchdog *watchdog, uint32_t now_ms)
{
    if (watchdog != NULL) {
        watchdog->last_valid_command_ms = now_ms;
        watchdog->armed = true;
    }
}

bool AmrWatchdog_IsTimedOut(
    const AmrCommandWatchdog *watchdog,
    uint32_t now_ms
)
{
    if ((watchdog == NULL) || !watchdog->armed) {
        return false;
    }
    return (uint32_t)(now_ms - watchdog->last_valid_command_ms)
        > watchdog->timeout_ms;
}
