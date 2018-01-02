#include <TeensyDmx.h>
#include <rdm.h>

#define DMX_REDE 2

// It was an easy job to register a manufacturer ID to myself as explained
// on http://tsp.esta.org/tsp/working_groups/CP/mfctrIDs.php.
// The first two bytes are the manufacturer ID, the next four must be unique
// for every device produced by that manufacturer.
// An easy way to do this is to use TeensyMAC to fetch the Teensy's unique
// MAC address, then map each OUI to a value in the third byte and use the
// three unique bytes of the MAC for bytes 4-6.
// The ID below is designated as a prototyping ID.
byte myUid[] = {0x7f, 0xf0, 0x20, 0x12, 0x00, 0x00};

// Broadcast to all devices, broadcasts don't generate a response
byte theirUid[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

enum Status { UN_MUTE, UN_MUTE_IDLE, DISCOVERY, DISCOVERY_IDLE, IDENTIFY_ON, IDENTIFY_ON_IDLE, IDENTIFY_OFF, IDENTIFY_OFF_IDLE };
Status currentState = UN_MUTE;
unsigned long lastAction = 0;

void discoveryComplete(CallbackStatus callbackStatus, byte *uids, uint32_t uidCount) {
  if (callbackStatus == CallbackStatus::CB_SUCCESS) {
    Serial.println("Printing UIDs:");

    Serial.print("Length: ");
    Serial.println(uidCount);

    for(uint32_t i = 0; i < uidCount; i++)
    {
      for(int j = 0; j < RDM_UID_LENGTH; j++)
      {
        Serial.print(uids[((i * RDM_UID_LENGTH) + j)], HEX);
        Serial.print(":");
      }
      Serial.println("");
    }
    Serial.println("");
    memcpy(theirUid, uids, RDM_UID_LENGTH);
    currentState = IDENTIFY_ON;
  } else {
    Serial.println("Discovery CB failed");
  }
}

void printRdm(CallbackStatus callbackStatus, RdmData *data) {
  if (callbackStatus == CallbackStatus::CB_SUCCESS) {
    Serial.println("Decoding RDM:");
    // TODO(Peter): Handle NAcks

    Serial.print("Length: ");
    Serial.println(data->length);
    Serial.print("Data Length: ");
    Serial.println(data->dataLength);
    
    Serial.print("Data: ");
    for(int i = 0; i < data->dataLength; i++)
    {
       Serial.print(char(data->data[i]));
    }
    Serial.println("");
  } else {
    Serial.println("Discovery CB failed");
  }
}

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
  0,   // Definition of additional commands
  &discoveryComplete,
  &printRdm
};

TeensyDmx Dmx(Serial1, &rdmData, DMX_REDE);

byte DMXVal[] = {50};

byte lowerUid[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte upperUid[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
byte broadcastUid[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void setup() {
  Dmx.setMode(TeensyDmx::DMX_OUT);
}

void loop() {
  //Dmx.setChannels(0, DMXVal, 1);
  switch (currentState)
  {
    case UN_MUTE:
      lastAction = millis();
      Dmx.sendRDMDiscUnMute(broadcastUid);
      currentState = UN_MUTE_IDLE;
      break;
    case UN_MUTE_IDLE:
      if ((millis() - lastAction) > 1000) {
        currentState = DISCOVERY;
      }
      break;
    case DISCOVERY:
      lastAction = millis();
      Dmx.sendRDMDiscUniqueBranch(lowerUid, upperUid);
      currentState = DISCOVERY_IDLE;
      break;
    case DISCOVERY_IDLE:
      //if ((millis() - lastAction) > 1000) {
      //  currentState = DISCOVERY;
      //}
      break;
    case IDENTIFY_ON:
      lastAction = millis();
      //Dmx.sendRDMSetIdentifyDevice(theirUid, true);
      Dmx.sendRDMGetManufacturerLabel(theirUid);
      //Dmx.sendRDMDiscUnMute(upperUid);
      currentState = IDENTIFY_ON_IDLE;
      break;
    case IDENTIFY_ON_IDLE:
      if ((millis() - lastAction) > 1000) {
        currentState = IDENTIFY_OFF;
      }
      break;
    case IDENTIFY_OFF:
      lastAction = millis();
      //Dmx.sendRDMSetIdentifyDevice(theirUid, false);
      Dmx.sendRDMGetManufacturerLabel(theirUid);
      //Dmx.sendRDMDiscUniqueBranch(lowerUid, upperUid);
      currentState = IDENTIFY_OFF_IDLE;
      break;
    case IDENTIFY_OFF_IDLE:
      if ((millis() - lastAction) > 1000) {
        //currentState = IDENTIFY_ON;
      }
      break;
  }
  Dmx.loop();
}
