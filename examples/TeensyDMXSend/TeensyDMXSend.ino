#include <TeensyDmx.h>

#define DMX_REDE 2

byte DMXVal[] = {50};

TeensyDmx Dmx(Serial1, DMX_REDE);

void setup() {
  Dmx.setMode(TeensyDmx::DMX_OUT);
}

void loop() {
  Dmx.setChannels(0, DMXVal, 1);
  Dmx.loop();
}
