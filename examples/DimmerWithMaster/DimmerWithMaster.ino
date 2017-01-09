/*********************************************************************
 * Dimmer Board (with Master Fader)
 **********************************************************************/

#include <TeensyDmx.h>

#define DMX_REDE 24

TeensyDmx Dmx(Serial1, DMX_REDE);

void setup() {
  analogReadRes(8);
  // Teensy DMX Declaration of Output
  Dmx.setMode(TeensyDmx::DMX_OUT);
}

void loop() {
  // Master fader
  float scalerVal = (floatmap(analogRead(9), 1, 1024, 1024, 1) / 1024);
  
  for (int i = 1; i <= 8; ++i) {   // Channel Fader 1-8
    byte dmxVal = round(floatmap(analogRead(i), 1, 1024, 255, 1) * scalerVal);
    Dmx.setDmxChannel(i, dmxVal);
  }
}

// function for mapping floats
float floatmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
