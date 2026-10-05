#ifndef APP_INTEGRATION_H
#define APP_INTEGRATION_H

#include <stdint.h>

void AppIntegration_Init(void);
void AppIntegration_Process(void);
void AppIntegration_OnControlTimerElapsed(void);
void AppIntegration_ForceSafeStop(void);

#endif
