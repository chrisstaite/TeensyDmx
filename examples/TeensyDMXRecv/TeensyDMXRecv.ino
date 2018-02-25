#include <TeensyDmx.h>
#include <rdm.h>

#define DMX_REDE 2
#define CHANNEL 0

// It was an easy job to register a manufacturer ID to myself as explained
// on http://tsp.esta.org/tsp/working_groups/CP/mfctrIDs.php.
// The first two bytes are the manufacturer ID, the next four must be unique
// for every device produced by that manufacturer.
// An easy way to do this is to use TeensyMAC to fetch the Teensy's unique
// MAC address, then map each OUI to a value in the third byte and use the
// three unique bytes of the MAC for bytes 4-6.
// The ID below is designated as a prototyping ID.
byte myUid[] = {0x7f, 0xf0, 0x20, 0x12, 0x00, 0x00};

struct RdmInit rdmData {
  myUid,
  0x00000100,
  "TeensyDMX v0.1",
  "Teensyduino",
  1,  // Device ID
  "DMX Node",
  E120_PRODUCT_CATEGORY_DIMMER_CS_LED,
  1,  // The DMX footprint
  1,  // The DMX startAddress - only used for RDM
  0,  // Additional commands length for RDM
  0   // Definition of additional commands
};

TeensyDmx Dmx(Serial1, &rdmData, DMX_REDE);

// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  Dmx.setMode(TeensyDmx::DMX_IN);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  Dmx.loop();
  if (Dmx.newFrame()) {
    Serial.println(Dmx.getBuffer()[CHANNEL]);
  }
  if (Dmx.rdmChanged()) {
    if (Dmx.isIdentify()) {
      Serial.println("Identify mode");
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
    Serial.print("Device label: ");
    Serial.println(Dmx.getLabel());

    Serial.print("Start address: ");
    Serial.println(rdmData.startAddress, DEC);

    Serial.print("Comms status - short messages: ");
    Serial.print(Dmx.getShortMessage(), DEC);
    Serial.print(", checksum fails: ");
    Serial.print(Dmx.getChecksumFail(), DEC);
    Serial.print(", length mismatches: ");
    Serial.println(Dmx.getLengthMismatch(), DEC);
  }
}
