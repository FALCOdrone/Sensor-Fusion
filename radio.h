#ifndef RADIO_H
#define RADIO_H

#include "pinDef.h"

#if defined USE_SBUS_RX
#include "SBUS.h"  //Bolder Flight SBUS v1.0.1
#endif

#if defined USE_DSM_RX
#include "src/DSMRX/DSMRX.h"
#endif

class radio {

    private:
        // Failsafe settings
        unsigned long radioFS[6] = {1000, 1500, 1500, 1500, 2000, 2000};
        unsigned minVal = 800;
        unsigned maxVal = 2200;

        unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
        unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
        int ppm_counter = 0;
        unsigned long time_ms = 0;

    public:

      void initializeRadio();
      void getCommands(unsigned long radioIn[], unsigned long radioInPrev[]);
      unsigned long getRadioPWM(int ch_num);
      void serialEvent3(void);
      void failSafe(unsigned long radioIn[]);

      #ifdef USE_PPM_RX
      void getPPM();
      #endif

      #ifdef USE_PPM_RX
      void getCh1();
      void getCh2();
      void getCh3();
      void getCh4();
      void getCh5();
      void getCh6();
      #endif

};

#endif