/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EVENTHANDLER_H
#define __EVENTHANDLER_H

#include "cmsis_os.h"

void setpointButtons();
void modeChangeButton();

osEventFlagsId_t setPointButtonEvents;
osEventFlagsId_t stateMachineEvents;

#endif /* __EVENTHANDLER_H*/
