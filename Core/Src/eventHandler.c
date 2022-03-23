#include "eventHandler.h"


void setpointButtons(){

	osEventFlagsSet( setPointButtonEvents, 0x50 );

}

void modeChangeButton(){

	osEventFlagsSet( stateMachineEvents, 0x69 );


}
