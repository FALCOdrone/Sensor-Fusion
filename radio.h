#ifndef RADIO_H
#define RADIO_H

#include "pinDef.h"

#if defined USE_SBUS_RX
#include "SBUS.h"  //Bolder Flight SBUS v1.0.1
#endif

#if defined USE_DSM_RX
#include "src/DSMRX/DSMRX.h"
#endif

void initializeRadio();
void getCommands(unsigned long radioIn[], unsigned long radioInPrev[]);

#endif