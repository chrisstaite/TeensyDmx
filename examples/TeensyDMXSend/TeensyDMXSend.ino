#include <TeensyDmx.h>

#define DMX_REDE 2

byte DMXVal[] = {50};

// This isn't required for DMX sending, but the code currently requires it.
struct RDMINIT rdmData {
  "TeensyDMX v0.1",
  "Teensyduino",
  1,  // Device ID
  "DMX Node",
  1,  // The DMX footprint
  0,  // The DMX startAddress - only used for RDM
  0,  // Additional commands length for RDM
  0   // Definition of additional commands
};

TeensyDmx Dmx(Serial1, &rdmData, DMX_REDE);

void setup() {
  Dmx.setMode(TeensyDmx::DMX_OUT);
}

void loop() {
  Dmx.setChannels(0, DMXVal, 1);
  Dmx.loop();
}
