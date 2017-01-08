#include "teensydmx.h"
#include "rdm.h"

#define BREAKSPEED     100000
#define RDM_BREAKSPEED 45500
#define BREAKFORMAT    SERIAL_8E1
#define DMXSPEED       250000
#define DMXFORMAT      SERIAL_8N2

#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))
#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)

// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))

// write a 16 bit number to a data buffer location
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);

typedef byte DEVICEID[6];

// It was an easy job to register a manufacturer id to myself as explained
// on http://tsp.plasa.org/tsp/working_groups/CP/mfctrIDs.php.
// The ID below is designated as a prototyping ID.
DEVICEID _devID = { 0x7f, 0xf0, 0x20, 0x12, 0x00, 0x00 };

// The Device ID for adressing all devices of a manufacturer.
DEVICEID _devIDGroup = { 0x7f, 0xf0, 0xFF, 0xFF, 0xFF, 0xFF };

// The Device ID for adressing all devices: 6 times 0xFF.
DEVICEID _devIDAll = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// compare 2 DeviceIDs
#define DeviceIDCmp(id1, id2) memcmp(id1, id2, sizeof(DEVICEID))

// copy an DeviceID id2 to id1
#define DeviceIDCpy(id1, id2) memcpy(id1, id2, sizeof(DEVICEID))

struct RDMDATA {
  //byte     StartCode;    // Start Code 0xCC for RDM (discarded by Recv)
  byte     SubStartCode; // Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[6];
  byte     SourceID[6];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType
  byte     _unknown;     // I don't know, ignore this
  uint16_t SubDev;      // sub device number (root = 0)
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[231];   // data byte field
}; // struct RDMDATA

// the special discovery response message
struct DISCOVERYMSG {
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
}; // struct DISCOVERYMSG

// The DEVICEINFO structure (length = 19) has to be responsed for E120_DEVICE_INFO
// See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DEVICEINFO {
  byte protocolMajor;
  byte protocolMinor;
  uint16_t deviceModel;
  uint16_t productCategory;
  uint32_t softwareVersion;
  uint16_t footprint;
  byte currentPersonality;
  byte personalityCount;
  uint16_t startAddress;
  uint16_t subDeviceCount;
  byte sensorCount;
}; // struct DEVICEINFO

// Instance for UART0, UART1, UART2
static TeensyDmx *uartInstances[3] = {0};

TeensyDmx::TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm, uint8_t redePin) :
    m_uart(uart),
    m_dmxBuffer1{0},
    m_dmxBuffer2{0},
    m_activeBuffer(m_dmxBuffer1),
    m_inactiveBuffer(m_dmxBuffer2),
    m_dmxBufferIndex(0),
    m_frameCount(0),
    m_newFrame(false),
    m_rdmChange(false),
    m_mode(DMX_OFF),
    m_state(State::IDLE),
    m_redePin(portOutputRegister(redePin)),
    m_rdmMute(false),
    m_identifyMode(false),
    m_rdm(rdm),
    m_deviceLabel{0}
{
    pinMode(redePin, OUTPUT);
    *m_redePin = 0;

    if (&m_uart == &Serial1) {
        uartInstances[0] = this;
    } else if (&m_uart == &Serial2) {
        uartInstances[1] = this;
    } else if (&m_uart == &Serial3) {
        uartInstances[2] = this;
    }
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, uint8_t redePin) :
    TeensyDmx(uart, nullptr, redePin)
{
}

const volatile uint8_t* TeensyDmx::getBuffer() const {
    return m_inactiveBuffer;
}

bool TeensyDmx::isIdentify() const {
    return m_identifyMode;
}

const char* TeensyDmx::getLabel() const {
    return m_deviceLabel;
}

void TeensyDmx::setMode(TeensyDmx::Mode mode) {
    switch (m_mode)
    {
        case DMX_IN:
            stopReceive();
            break;
        case DMX_OUT:
            stopTransmit();
            break;
        default:
            // No action
            break;
    }

    m_mode = mode;

    switch (m_mode)
    {
        case DMX_IN:
            startReceive();
            break;
        case DMX_OUT:
            startTransmit();
            break;
        default:
            *m_redePin = 0;  // Off puts in receive state so as to be passive
            break;
    }
}

void TeensyDmx::setChannel(const uint16_t address, const uint8_t value) {
    if (address < DMX_BUFFER_SIZE) {
        m_activeBuffer[address] = value;
    }
}

void TeensyDmx::setChannels(const uint16_t startAddress, const uint8_t* values, const uint16_t length) {
    uint16_t correctedLength;
    if (startAddress + length > DMX_BUFFER_SIZE) {
        correctedLength = DMX_BUFFER_SIZE - startAddress;
    } else {
        correctedLength = length;
    }

    if (startAddress > 0) {
        memset((void*)m_activeBuffer, 0, startAddress);
    }
    memcpy((void*)(m_activeBuffer + startAddress), values, correctedLength);
    if (startAddress + correctedLength != DMX_BUFFER_SIZE) {
        memset((void*)(m_activeBuffer + startAddress + length), 0, DMX_BUFFER_SIZE - correctedLength);
    }
}

void TeensyDmx::nextTx()
{
    if (m_state == State::BREAK) {
        m_state = DMX_TX;
        m_uart.begin(DMXSPEED, DMXFORMAT);
        m_uart.write(0);
    } else if (m_state == State::DMX_TX) {
        // Check if we're at the end of the packet
        if (m_dmxBufferIndex == DMX_BUFFER_SIZE) {
            // Send BREAK
            m_state = State::BREAK;
            m_uart.begin(BREAKSPEED, BREAKFORMAT);
            m_uart.write(0);
        } else {
            m_uart.write(m_activeBuffer[m_dmxBufferIndex++]);
        }
    }
}

void uart0_status_isr();  // Back reference to serial1.c
void UART0TxStatus()
{
    if ((UART0_C2 & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[0]->nextTx();
    }
    // Call standard ISR too
    uart0_status_isr();
}

void uart1_status_isr();  // Back reference to serial2.c
void UART1TxStatus()
{
    if ((UART1_C2 & UART_C2_TCIE) && (UART1_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[1]->nextTx();
    }
    // Call standard ISR too
    uart1_status_isr();
}

void uart2_status_isr();  // Back reference to serial3.c
void UART2TxStatus()
{
    if ((UART2_C2 & UART_C2_TCIE) && (UART2_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[2]->nextTx();
    }
    // Call standard ISR too
    uart2_status_isr();
}

void uart0_error_isr();  // Back reference to serial1.c
// UART0 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART0RxError(void)
{
    // On break, uart0_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART0_S1 & UART_S1_FE)
        (void) UART0_D;

    uartInstances[0]->completeFrame();
}

void uart1_error_isr();  // Back reference to serial2.c
// UART1 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART1RxError(void)
{
    // On break, uart1_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART1_S1 & UART_S1_FE)
        (void) UART1_D;

    uartInstances[1]->completeFrame();
}

void uart2_error_isr();  // Back reference to serial3.c
// UART2 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART2RxError(void)
{
    // On break, uart2_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART2_S1 & UART_S1_FE)
        (void) UART2_D;

    uartInstances[2]->completeFrame();
}

void TeensyDmx::startTransmit()
{
    *m_redePin = 1;

    m_dmxBufferIndex = 0;

    if (&m_uart == &Serial1) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART0_STATUS, UART0TxStatus);
        attachInterruptVector(IRQ_UART0_ERROR, UART0RxError);
    } else if (&m_uart == &Serial2) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART1_STATUS, UART1TxStatus);
        attachInterruptVector(IRQ_UART1_ERROR, UART1RxError);
    } else if (&m_uart == &Serial3) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART2_STATUS, UART2TxStatus);
        attachInterruptVector(IRQ_UART2_ERROR, UART2RxError);
    }

    // Send BREAK
    m_state = State::BREAK;
    m_uart.begin(BREAKSPEED, BREAKFORMAT);
    m_uart.write(0);

}

void TeensyDmx::stopTransmit()
{
    m_uart.end();

    if (&m_uart == &Serial1) {
        attachInterruptVector(IRQ_UART0_STATUS, uart0_status_isr);
        attachInterruptVector(IRQ_UART0_ERROR, uart0_error_isr);
    } else if (&m_uart == &Serial2) {
        attachInterruptVector(IRQ_UART1_STATUS, uart1_status_isr);
        attachInterruptVector(IRQ_UART1_ERROR, uart1_error_isr);
    } else if (&m_uart == &Serial3) {
        attachInterruptVector(IRQ_UART2_STATUS, uart2_status_isr);
        attachInterruptVector(IRQ_UART2_ERROR, uart2_error_isr);
    }
}

bool TeensyDmx::newFrame(void)
{
    if (m_newFrame) {
        m_newFrame = false;
        return true;
    }
    return false;
}

bool TeensyDmx::rdmChanged(void)
{
    if (m_rdmChange) {
        m_rdmChange = false;
        return true;
    }
    return false;
}

void TeensyDmx::completeFrame()
{
    // Ensure we've processed all the data that may still be sitting
    // in software buffers.
    readBytes();

    if (m_state == State::DMX_RECV || m_state == State::DMX_COMPLETE) {
        // Update frame count and swap buffers
        ++m_frameCount;
        if (m_activeBuffer == m_dmxBuffer1) {
            m_activeBuffer = m_dmxBuffer2;
            m_inactiveBuffer = m_dmxBuffer1;
        } else {
            m_activeBuffer = m_dmxBuffer1;
            m_inactiveBuffer = m_dmxBuffer2;
        }
        m_newFrame = true;
    } else if (m_state == State::RDM_RECV) {
        // Check if we need to reply to this RDM message
        m_state = State::IDLE; // Stop the ISR messing up things
        processRDM();
    }
    m_dmxBufferIndex = 0;
    m_state = State::BREAK;
}

void TeensyDmx::processRDM()
{
    struct RDMDATA* rdm = (struct RDMDATA*)(m_activeBuffer);
    unsigned long timingStart = micros();

    bool isForMe = !DeviceIDCmp(rdm->DestID, _devID);
    if (!isForMe
            && DeviceIDCmp(rdm->DestID, _devIDAll)
            && DeviceIDCmp(rdm->DestID, _devIDGroup)) {
        // This packet is not for me...
        return;
    }

    bool handled = false;
    bool shouldRespond = isForMe;
    uint16_t nackReason = E120_NR_UNKNOWN_PID;

    switch (rdm->CmdClass) {
        case E120_DISCOVERY_COMMAND:
            shouldRespond = false;
            if (rdm->Parameter == SWAPINT(E120_DISC_UNIQUE_BRANCH)) {
                if (m_rdmMute) return;
                
                if (rdm->Length != 36) return;
                if (rdm->DataLength != 12) return;
                
                if ((DeviceIDCmp(rdm->Data, _devID) <= 0) && (DeviceIDCmp(_devID, rdm->Data+6) <= 0)) {
                    // I'm in range - say hello to the lovely controller
                    
                    // respond a special discovery message !
                    struct DISCOVERYMSG *disc = (struct DISCOVERYMSG*)(m_activeBuffer);
                    uint16_t checksum = 6 * 0xFF;
              
                    // fill in the _rdm.discovery response structure
                    for (byte i = 0; i < 7; i++) {
                        disc->headerFE[i] = 0xFE;
                    }
                    disc->headerAA = 0xAA;
                    for (byte i = 0; i < 6; i++) {
                        disc->maskedDevID[i+i]   = _devID[i] | 0xAA;
                        disc->maskedDevID[i+i+1] = _devID[i] | 0x55;
                        checksum += _devID[i];
                    }
                    disc->checksum[0] = (checksum >> 8)   | 0xAA;
                    disc->checksum[1] = (checksum >> 8)   | 0x55;
                    disc->checksum[2] = (checksum & 0xFF) | 0xAA;
                    disc->checksum[3] = (checksum & 0xFF) | 0x55;
                    
                    // Send reply
                    stopReceive();
                    *m_redePin = 1;
                    m_dmxBufferIndex = 0;
                    m_uart.begin(RDM_BREAKSPEED, BREAKFORMAT);
                    m_uart.write(0);
                    m_uart.flush();
                    m_uart.begin(DMXSPEED, DMXFORMAT);
                    for (uint16_t i = 0; i < sizeof(struct DISCOVERYMSG); ++i) {
                        m_uart.write(m_activeBuffer[i]);
                        m_uart.flush();
                    }
                    startReceive();
                }
            } else if (rdm->Parameter == SWAPINT(E120_DISC_UN_MUTE)) { // 0x0003
                handled = true;
                if (isForMe) {
                    if (rdm->DataLength > 0) {
                        // Unexpected data
                        // Do nothing
                        shouldRespond = false;
                    } else {
                        m_rdmMute = false;
                        // Control field
                        rdm->Data[0] = 0b00000000;
                        rdm->Data[1] = 0b00000000;
                        rdm->DataLength = 2;
                        shouldRespond = true;
                    }
                }
            } else if (rdm->Parameter == SWAPINT(E120_DISC_MUTE)) { // 0x0002
                handled = true;
                if (isForMe) {
                    if (rdm->DataLength > 0) {
                        // Unexpected data
                        // Do nothing
                        shouldRespond = false;
                    } else {
                        m_rdmMute = true;
                        // Control field
                        rdm->Data[0] = 0b00000000;
                        rdm->Data[1] = 0b00000000;
                        rdm->DataLength = 2;
                        shouldRespond = true;
                    }
                }
            }
            break;
        case E120_SET_COMMAND:
            if (rdm->Parameter == SWAPINT(E120_IDENTIFY_DEVICE)) {
                if (rdm->DataLength != 1) {
                    // Oversized data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if ((rdm->Data[0] != 0) && (rdm->Data[0] != 1)) {
                    // Out of range data
                    nackReason = E120_NR_DATA_OUT_OF_RANGE;
                } else {
                  m_identifyMode = rdm->Data[0] != 0;
                  rdm->DataLength = 0;
                  handled = true;
                  m_rdmChange = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DEVICE_LABEL)) {
                if (rdm->DataLength > sizeof(m_deviceLabel)) {
                    // Oversized data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else {
                    memcpy(m_deviceLabel, rdm->Data, rdm->DataLength);
                    m_deviceLabel[rdm->DataLength] = '\0';
                    rdm->DataLength = 0;
                    handled = true;
                    m_rdmChange = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DMX_START_ADDRESS)) {
                if (rdm->DataLength != 2) {
                    // Oversized data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else {
                    uint16_t newStartAddress = READINT(rdm->Data);
                    if ((newStartAddress <= 0) || (newStartAddress > DMX_BUFFER_SIZE)) {
                        // Out of range start address
                        nackReason = E120_NR_DATA_OUT_OF_RANGE;
                    } else if (m_rdm == nullptr) {
                        nackReason = E120_NR_HARDWARE_FAULT;
                    } else {
                        m_rdm->startAddress = newStartAddress;
                        rdm->DataLength = 0;
                        handled = true;
                        m_rdmChange = true;
                    }
                }
            } else if (rdm->Parameter == SWAPINT(E120_SUPPORTED_PARAMETERS)) {
                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
            }
            break;
        case E120_GET_COMMAND:
            if (rdm->Parameter == SWAPINT(E120_IDENTIFY_DEVICE)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else {
                  rdm->Data[0] = m_identifyMode;
                  rdm->DataLength = 1;
                  handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DEVICE_INFO)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else {
                    // return all device info data
                    DEVICEINFO *devInfo = (DEVICEINFO *)(rdm->Data); // The data has to be responsed in the Data buffer.

                    devInfo->protocolMajor = 1;
                    devInfo->protocolMinor = 0;
                    devInfo->productCategory = SWAPINT(E120_PRODUCT_CATEGORY_DIMMER_CS_LED);
                    devInfo->softwareVersion = SWAPINT32(0x01000000);// 0x04020900;
                    devInfo->currentPersonality = 1;
                    devInfo->personalityCount = 1;
                    devInfo->subDeviceCount = 0;
                    devInfo->sensorCount = 0;
                    if (m_rdm == nullptr) {
                        devInfo->deviceModel = 0;
                        devInfo->startAddress = 0;
                        devInfo->footprint = 0;
                    } else {
                        devInfo->deviceModel = SWAPINT(m_rdm->deviceModelId);
                        devInfo->startAddress = SWAPINT(m_rdm->startAddress);
                        devInfo->footprint = SWAPINT(m_rdm->footprint);
                    }

                    rdm->DataLength = sizeof(DEVICEINFO);
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_MANUFACTURER_LABEL)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else if (m_rdm == nullptr) {
                    rdm->DataLength = 0;
                    handled = true;
                } else {
                    // return the manufacturer label
                    rdm->DataLength = strlen(m_rdm->manufacturerLabel);
                    memcpy(rdm->Data, m_rdm->manufacturerLabel, rdm->DataLength);
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DEVICE_MODEL_DESCRIPTION)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else if (m_rdm == nullptr) {
                    rdm->DataLength = 0;
                    handled = true;
                } else {
                    // return the DEVICE MODEL DESCRIPTION
                    rdm->DataLength = strlen(m_rdm->deviceModel);
                    memcpy(rdm->Data, m_rdm->deviceModel, rdm->DataLength);
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DEVICE_LABEL)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else {
                    rdm->DataLength = strlen(m_deviceLabel);
                    memcpy(rdm->Data, m_deviceLabel, rdm->DataLength);
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_SOFTWARE_VERSION_LABEL)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else if (m_rdm == nullptr) {
                    rdm->DataLength = 0;
                    handled = true;
                } else {
                    // return the SOFTWARE_VERSION_LABEL
                    rdm->DataLength = strlen(m_rdm->softwareLabel);
                    memcpy(rdm->Data, m_rdm->softwareLabel, rdm->DataLength);
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_DMX_START_ADDRESS)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else {
                    if (m_rdm == nullptr) {
                        WRITEINT(rdm->Data, 0);
                    } else {
                        WRITEINT(rdm->Data, m_rdm->startAddress);
                    }
                    rdm->DataLength = 2;
                    handled = true;
                }
            } else if (rdm->Parameter == SWAPINT(E120_SUPPORTED_PARAMETERS)) {
                if (rdm->DataLength > 0) {
                    // Unexpected data
                    nackReason = E120_NR_FORMAT_ERROR;
                } else if (rdm->SubDev != 0) {
                    // No sub-devices supported
                    nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
                } else {
                    if (m_rdm == nullptr) {
                        rdm->DataLength = 6;
                    } else {
                        rdm->DataLength = 2 * (3 + m_rdm->additionalCommandsLength);
                        for (int n = 0; n < m_rdm->additionalCommandsLength; ++n) {
                            WRITEINT(rdm->Data+6+n+n, m_rdm->additionalCommands[n]);
                        }
                    }
                    WRITEINT(rdm->Data,   E120_MANUFACTURER_LABEL);
                    WRITEINT(rdm->Data+2, E120_DEVICE_MODEL_DESCRIPTION);
                    WRITEINT(rdm->Data+4, E120_DEVICE_LABEL);
                    handled = true;
                }
            }
            break;
        default:
            break;
    }

    if (shouldRespond) {
        // TIMING: don't send too fast, min: 176 microseconds
        timingStart = micros() - timingStart;
        if (timingStart < 176) {
            delayMicroseconds(176 - timingStart);
        }
        respondMessage(handled, nackReason);
    }
}

void TeensyDmx::respondMessage(bool isHandled, uint16_t nackReason)
{
    uint16_t i;
    uint16_t checkSum = 0;
    struct RDMDATA* rdm = (struct RDMDATA*)(m_activeBuffer);

    // no need to set these data fields:
    // StartCode, SubStartCode
    if (isHandled) {
        rdm->ResponseType = E120_RESPONSE_TYPE_ACK; // 0x00
    } else {
        rdm->ResponseType = E120_RESPONSE_TYPE_NACK_REASON; // 0x00
        rdm->DataLength = 2;
        rdm->Data[0] = (nackReason >> 8) & 0xFF;
        rdm->Data[1] = nackReason & 0xFF;
    }
    rdm->Length = rdm->DataLength + 24; // total packet length

    // swap SrcID into DestID for sending back.
    DeviceIDCpy(rdm->DestID, rdm->SourceID);
    DeviceIDCpy(rdm->SourceID, _devID);

    ++(rdm->CmdClass);
    // Parameter

    // prepare buffer and Checksum
    checkSum += E120_SC_RDM;
    for (i = 0; i < rdm->Length; i++) {
        checkSum += m_activeBuffer[i];
    }

    // Send reply
    stopReceive();
    *m_redePin = 1;
    m_dmxBufferIndex = 0;

    m_uart.begin(RDM_BREAKSPEED, BREAKFORMAT);
    m_uart.write(0);
    m_uart.flush();
    m_uart.begin(DMXSPEED, DMXFORMAT);
    m_uart.write(E120_SC_RDM);
    m_uart.flush();
    for (uint16_t i = 0; i < rdm->Length; ++i) {
        m_uart.write(m_activeBuffer[i]);
        m_uart.flush();
    }
    m_uart.write(checkSum >> 8);
    m_uart.flush();
    m_uart.write(checkSum & 0xff);
    m_uart.flush();

    // Restart receive
    startReceive();
}

void TeensyDmx::startReceive()
{
    *m_redePin = 0;

    // UART Initialisation
    m_uart.begin(250000);

    if (&m_uart == &Serial1) {
        // Fire UART0 receive interrupt immediately after each byte received
        UART0_RWFIFO = 1;
        
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART0_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART0_STATUS) + 1);

        // Enable UART0 interrupt on frame error and enable IRQ
        UART0_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
    } else if (&m_uart == &Serial2) {
        // Fire UART1 receive interrupt immediately after each byte received
        UART1_RWFIFO = 1;

        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART1_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART1_STATUS) + 1);

        // Enable UART1 interrupt on frame error and enable IRQ
        UART1_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
    } else if (&m_uart == &Serial3) {
        // Fire UART2 receive interrupt immediately after each byte received
        UART2_RWFIFO = 1;
        
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART2_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART2_STATUS) + 1);

        // Enable UART2 interrupt on frame error and enable IRQ
        UART2_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);

    }

    m_dmxBufferIndex = 0;
    m_state = State::IDLE;
}

void TeensyDmx::stopReceive()
{
    m_uart.end();
    if (&m_uart == &Serial1) {
        UART0_RWFIFO = 0;
        UART0_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
    } else if (&m_uart == &Serial2) {
        UART1_RWFIFO = 0;
        UART1_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
    } else if (&m_uart == &Serial3) {
        UART2_RWFIFO = 0;
        UART2_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
    }
}

void TeensyDmx::readBytes()
{
    __disable_irq();  // Prevents conflicts with the error ISR

    int available = m_uart.available();
    while (available--) {
        switch (m_state) {
            case State::BREAK:
                switch (m_uart.read()) {
                    case 0:
                        m_state = State::DMX_RECV;
                        break;
                    case E120_SC_RDM:
                        m_state = State::RDM_RECV;
                        break;
                    default:
                        m_state = State::IDLE;
                        break;
                }
                break;
            case State::RDM_RECV:
            case State::DMX_RECV:
                m_activeBuffer[++m_dmxBufferIndex] = m_uart.read();

                if (m_dmxBufferIndex == DMX_BUFFER_SIZE) {
                    if (m_state == State::DMX_RECV) {
                        m_state = State::DMX_COMPLETE;
                    } else {
                        m_state = State::IDLE;  // Buffer full
                    }
                }
                break;
            default:
                // Discarding bytes
                m_uart.read();
                break;
        }
    }
    __enable_irq();
}

void TeensyDmx::loop()
{
    if (m_mode == DMX_IN) {
        readBytes();
    }
}
