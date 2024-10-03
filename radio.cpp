#include "radio.h"

#if defined USE_SBUS_RX
SBUS sbus(Serial6);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
DSM1024 DSM;
#endif

void Radio::initializeRadio() {
    // PPM Receiver
#if defined USE_PPM_RX
    // Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    // Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

// PWM Receiver
#elif defined USE_PWM_RX
    // Declare interrupt pins
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    // Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

// SBUS Recevier
#elif defined USE_SBUS_RX
    sbus.begin();

// DSM receiver
#elif defined USE_DSM_RX
    Serial3.begin(115000);
#else
#error No RX type defined...
#endif

}

unsigned long Radio::getRadioPWM(int ch_num) {
    // DESCRIPTION: Get current radio commands from interrupt routines
    unsigned long returnPWM = 0;

    if (ch_num == 1) {
        returnPWM = channel_1_raw;
    } else if (ch_num == 2) {
        returnPWM = channel_2_raw;
    } else if (ch_num == 3) {
        returnPWM = channel_3_raw;
    } else if (ch_num == 4) {
        returnPWM = channel_4_raw;
    } else if (ch_num == 5) {
        returnPWM = channel_5_raw;
    } else if (ch_num == 6) {
        returnPWM = channel_6_raw;
    }

    return returnPWM;
}

void Radio::serialEvent3(void) {
#if defined USE_DSM_RX
    while (Serial3.available()) {
        DSM.handleSerialEvent(Serial3.read(), micros());
    }
#endif
}

#ifdef USE_PPM_RX
void radio::getPPM() {
    unsigned long dt_ppm;
    int trig = digitalRead(PPM_Pin);
    if (trig == 1) {  // Only care about rising edge
        dt_ppm = micros() - time_ms;
        time_ms = micros();

        if (dt_ppm > 5000) {  // Waiting for long pulse to indicate a new pulse train has arrived
            ppm_counter = 0;
        }

        if (ppm_counter == 1) {  // First pulse
            channel_1_raw = dt_ppm;
        }

        if (ppm_counter == 2) {  // Second pulse
            channel_2_raw = dt_ppm;
        }

        if (ppm_counter == 3) {  // Third pulse
            channel_3_raw = dt_ppm;
        }

        if (ppm_counter == 4) {  // Fourth pulse
            channel_4_raw = dt_ppm;
        }

        if (ppm_counter == 5) {  // Fifth pulse
            channel_5_raw = dt_ppm;
        }

        if (ppm_counter == 6) {  // Sixth pulse
            channel_6_raw = dt_ppm;
        }

        ppm_counter = ppm_counter + 1;
    }
}
#endif

#ifdef USE_PPM_RX
void radio::getCh1() {
    int trigger = digitalRead(ch1Pin);
    if (trigger == 1) {
        rising_edge_start_1 = micros();
    } else if (trigger == 0) {
        channel_1_raw = micros() - rising_edge_start_1;
    }
}

void radio::getCh2() {
    int trigger = digitalRead(ch2Pin);
    if (trigger == 1) {
        rising_edge_start_2 = micros();
    } else if (trigger == 0) {
        channel_2_raw = micros() - rising_edge_start_2;
    }
}

void radio::getCh3() {
    int trigger = digitalRead(ch3Pin);
    if (trigger == 1) {
        rising_edge_start_3 = micros();
    } else if (trigger == 0) {
        channel_3_raw = micros() - rising_edge_start_3;
    }
}

void radio::getCh4() {
    int trigger = digitalRead(ch4Pin);
    if (trigger == 1) {
        rising_edge_start_4 = micros();
    } else if (trigger == 0) {
        channel_4_raw = micros() - rising_edge_start_4;
    }
}

void radio::getCh5() {
    int trigger = digitalRead(ch5Pin);
    if (trigger == 1) {
        rising_edge_start_5 = micros();
    } else if (trigger == 0) {
        channel_5_raw = micros() - rising_edge_start_5;
    }
}

void radio::getCh6() {
    int trigger = digitalRead(ch6Pin);
    if (trigger == 1) {
        rising_edge_start_6 = micros();
    } else if (trigger == 0) {
        channel_6_raw = micros() - rising_edge_start_6;
    }
}
#endif

void Radio::failSafe(unsigned long radioIn[]) {
    // DESCRIPTION: If radio gives garbage values, set all commands to default values
    /*
     * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
     * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
     * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
     * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
     * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
     */

    int check[6] = {0, 0, 0, 0, 0, 0};

    // Triggers for failure criteria
    for (int i = 0; i < 6; i++) {
        if (radioIn[i] > maxVal || radioIn[i] < minVal) {
            check[i] = 1;
        }
    }

    // If any failures, set to default failsafe values
    if ((check[0] + check[1] + check[2] + check[3] + check[4] + check[5]) > 0) {
        for (int i = 0; i < 6; i++) {
            radioIn[i] = radioFS[i];
        }
    }
}

void Radio::getCommands(unsigned long radioIn[], unsigned long radioInPrev[]) {
    // DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
     * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
     * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
     * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
     * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
     */

#if defined USE_PPM_RX || defined USE_PWM_RX
    for (int i = 0; i < 6; i++) {
        radioIn[i] = getRadioPWM(i + 1);
    }

#elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
        // sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;
        float bias = 895.0;
        for (int i = 0; i < 6; i++) {
            radioIn[i] = sbusChannels[i] * scale + bias;
        }
    }

#elif defined USE_DSM_RX
    if (DSM.timedOut(micros())) {
        // Serial.println("*** DSM RX TIMED OUT ***");
    } else if (DSM.gotNewFrame()) {
        uint16_t values[num_DSM_channels];
        DSM.getChannelValues(values, num_DSM_channels);

        for (int i = 0; i < 6; i++) {
            radioInPWM[i] = values[i];
        }
    }
#endif

    // Low-pass the critical commands (0-3) and update previous values
    float b = 0.7;  // Lower=slower, higher=noiser
    for (int i = 0; i <= 3; i++) {
        radioIn[i] = (1.0 - b) * radioInPrev[i] + b * radioIn[i];
        radioInPrev[i] = radioIn[i];
    }
    failSafe(radioIn);  // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
}