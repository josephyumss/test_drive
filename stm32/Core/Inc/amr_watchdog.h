#ifndef AMR_WATCHDOG_H
#define AMR_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timeout_ms;
    uint32_t last_valid_command_ms;
    bool armed;
} AmrCommandWatchdog;

void AmrWatchdog_Init(AmrCommandWatchdog *watchdog, uint32_t timeout_ms);
void AmrWatchdog_Kick(AmrCommandWatchdog *watchdog, uint32_t now_ms);
bool AmrWatchdog_IsTimedOut(
    const AmrCommandWatchdog *watchdog,
    uint32_t now_ms
);

#endif
