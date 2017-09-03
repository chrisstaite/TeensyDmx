#include <TeensyDmx.h>

#define DMX_REDE 2
#define CHANNEL 0

struct RDMINIT rdmData {
  "TeensyDMX v0.1",
  "Teensyduino",
  1,  // Device ID
  "DMX Node",
  1,  // The DMX footprint
  1,  // The DMX startAddress - only used for RDM
  0,  // Additional commands length for RDM
  0   // Definition of additional commands
};

byte DMXVal[] = {50};

TeensyDmx Dmx(Serial1, &rdmData, DMX_REDE);

void setup() {
  Serial.begin(9600);
  Dmx.setMode(TeensyDmx::DMX_IN);
}

void loop() {
  Dmx.loop();
  if (Dmx.newFrame()) {
    Serial.println(Dmx.getBuffer()[CHANNEL]);
  }
  if (Dmx.rdmChanged()) {
    if (Dmx.isIdentify()) {
      Serial.println("Identify mode");
    }
    Serial.print("Device label: ");
    Serial.println(Dmx.getLabel());
    Serial.print("Start address: ");
    Serial.println(rdmData.startAddress, DEC);
  }
}
