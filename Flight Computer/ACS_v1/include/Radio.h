#pragma once


#include <Arduino.h>
#include "Config.h"


unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6, rising_edge_start_7, rising_edge_start_8;
int ppm_counter = 0;
unsigned long time_ms = 0;

unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw, channel_7_raw, channel_8_raw;


void getCh1() 
{
    int trigger = digitalRead(CH1_PIN);

    if(trigger == 1) {
        rising_edge_start_1 = micros();
    }
    else if(trigger == 0) {
        channel_1_raw = micros() - rising_edge_start_1;
    }
}

void getCh2() {
  int trigger = digitalRead(CH2_PIN);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(CH3_PIN);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(CH4_PIN);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(CH5_PIN);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

void getCh6() {
  int trigger = digitalRead(CH6_PIN);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}

void getCh7() {
  int trigger = digitalRead(CH7_PIN);
  if(trigger == 1) {
    rising_edge_start_7 = micros();
  }
  else if(trigger == 0) {
    channel_7_raw = micros() - rising_edge_start_7;
  }
}

void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_PIN);
  if (trig==1) { //only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //first pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}


class Radio
{
private:
    
public:
    Radio();
    ~Radio();
    unsigned long getRadioPWM(int ch_num);
    void radioSetup();
};

Radio::Radio(){}

Radio::~Radio(){}

void Radio::radioSetup() 
{
    //PPM Receiver 
  #if defined USE_PPM_RX
    //Declare interrupt pin
    pinMode(PPM_PIN, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), getPPM, CHANGE);

  //PWM Receiver
  #elif defined USE_PWM_RX
    //Declare interrupt pins 
    pinMode(CH1_PIN, INPUT_PULLUP);
    pinMode(CH2_PIN, INPUT_PULLUP);
    pinMode(CH3_PIN, INPUT_PULLUP);
    pinMode(CH4_PIN, INPUT_PULLUP);
    pinMode(CH5_PIN, INPUT_PULLUP);
    pinMode(CH6_PIN, INPUT_PULLUP);
    pinMode(CH7_PIN, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(CH1_PIN), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH2_PIN), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH3_PIN), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH4_PIN), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH5_PIN), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH6_PIN), getCh6, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH7_PIN), getCh7, CHANGE);
    delay(20);

  //SBUS Recevier 
  #elif defined USE_SBUS_RX
    // sbus.begin();
  #else
    #error No RX type defined...
  #endif

}

unsigned long Radio::getRadioPWM(int ch_num)
{
    unsigned long returnPWM;

    if (ch_num == 1) {
        returnPWM = channel_1_raw;
    }
    else if (ch_num == 2) {
        returnPWM = channel_2_raw;
    }
    else if (ch_num == 3) {
        returnPWM = channel_3_raw;
    }
    else if (ch_num == 4) {
        returnPWM = channel_4_raw;
    }
    else if (ch_num == 5) {
        returnPWM = channel_5_raw;
    }
    else if (ch_num == 6) {
        returnPWM = channel_6_raw;
    }
    else if (ch_num == 7) {
        returnPWM = channel_7_raw;
    }

    return returnPWM;
}