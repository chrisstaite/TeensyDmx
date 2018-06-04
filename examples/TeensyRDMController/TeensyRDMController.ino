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

#define MAX_UID_COUNT 40

byte foundUids[RDM_UID_LENGTH * MAX_UID_COUNT];
uint32_t uidNum = 0;

// Broadcast to all devices, broadcasts don't generate a response
byte theirUid[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

enum Status { DISCOVERY, DISCOVERY_IDLE, SWITCH_UID, IDENTIFY_ON, IDENTIFY_ON_IDLE, IDENTIFY_OFF, IDENTIFY_OFF_IDLE };
Status currentState = DISCOVERY;
unsigned long lastAction = 0;

void discoveryComplete(CallbackStatus callbackStatus, byte *uids, uint32_t uidCount) {
  if (callbackStatus == CallbackStatus::CB_SUCCESS) {
    if (uidCount > 0) {
      Serial.println("Printing UIDs:");

      Serial.print("Length: ");
      Serial.println(uidCount);

      for(uint32_t i = 0; i < uidCount; i++)
      {
        for(int j = 0; j < RDM_UID_LENGTH; j++)
        {
          Serial.print(uids[((i * RDM_UID_LENGTH) + j)], HEX);
          if ((j + 1) < RDM_UID_LENGTH) {
              // Don't print a colon after the last byte
              Serial.print(":");
          }
        }
        Serial.println("");
      }
      Serial.println("");
      memcpy(foundUids, uids, (uidCount * RDM_UID_LENGTH));
      uidNum = uidCount;
      currentState = SWITCH_UID;
    } else {
      Serial.println("No UIDs returned");
    }
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
  } else if (callbackStatus == CallbackStatus::CB_RDM_BROADCAST) {
    Serial.println("RDM message was broadcast, no response expected");
  } else if (callbackStatus == CallbackStatus::CB_RDM_TIMEOUT) {
    Serial.println("RDM message timed out");
  } else {
    Serial.println("RDM CB failed");
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

void setup() {
  Dmx.setMode(TeensyDmx::DMX_OUT);
}

void loop() {
  //Dmx.setChannels(0, DMXVal, 1);
  switch (currentState)
  {
    case DISCOVERY:
      Dmx.doRDMDiscovery();
      currentState = DISCOVERY_IDLE;
      break;
    case DISCOVERY_IDLE:
      //if ((millis() - lastAction) > 1000) {
      //  currentState = DISCOVERY;
      //}
      break;
    case SWITCH_UID:
      if (uidNum > 0) {
        // Move onto the next UID
        --uidNum;
        memcpy(theirUid, &foundUids[(uidNum * RDM_UID_LENGTH)], RDM_UID_LENGTH);

        Serial.print("Fetching info for ");
        for(int i = 0; i < RDM_UID_LENGTH; i++)
        {
          Serial.print(theirUid[i], HEX);
          if ((i + 1) < RDM_UID_LENGTH) {
            // Don't print a colon after the last byte
            Serial.print(":");
          }
        }
        Serial.println("");
      }
      currentState = IDENTIFY_ON;
      break;
    case IDENTIFY_ON:
      lastAction = millis();
      //Dmx.sendRDMSetIdentifyDevice(theirUid, true);
      Dmx.sendRDMGetManufacturerLabel(theirUid);
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
      Dmx.sendRDMGetDeviceModelDescription(theirUid);
      currentState = IDENTIFY_OFF_IDLE;
      break;
    case IDENTIFY_OFF_IDLE:
      if ((millis() - lastAction) > 1000) {
        if (uidNum > 0) {
          currentState = SWITCH_UID;
        } else {
          //currentState = IDENTIFY_ON;
        }
      }
      break;
  }
  Dmx.loop();
}
