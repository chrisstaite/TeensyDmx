#include "TeensyDmx.h"
#include "rdm.h"

static constexpr uint32_t BREAKSPEED = 100000;
static constexpr uint32_t RDM_BREAKSPEED = 45500;
static constexpr uint32_t BREAKFORMAT = SERIAL_8E1;
static constexpr uint32_t DMXSPEED = 250000;
static constexpr uint32_t DMXFORMAT = SERIAL_8N2;
static constexpr uint16_t NACK_WAS_ACK = 0xffff;  // Send an ACK, not a NACK

// It was an easy job to register a manufacturer id to myself as explained
// on http://tsp.plasa.org/tsp/working_groups/CP/mfctrIDs.php.
// The ID below is designated as a prototyping ID.
static constexpr byte _devID[] = { 0x7f, 0xf0, 0x20, 0x12, 0x00, 0x00 };

// The Device ID for adressing all devices of a manufacturer.
static constexpr byte _devIDGroup[] = { 0x7f, 0xf0, 0xFF, 0xFF, 0xFF, 0xFF };

// The Device ID for adressing all devices: 6 times 0xFF.
static constexpr byte _devIDAll[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

struct RDMDATA
{
  //byte     StartCode;    // Start Code 0xCC for RDM (discarded by Recv)
  byte     SubStartCode; // Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[6];
  byte     SourceID[6];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType or PortID
  byte     MessageCount;     // number of queued messages
  uint16_t SubDev;      // sub device number (root = 0)
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[231];   // data byte field
}; // struct RDMDATA

// the special discovery response message
struct DISCOVERYMSG
{
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
}; // struct DISCOVERYMSG

// The DEVICEINFO structure (length = 19) has to be responsed for E120_DEVICE_INFO
// See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DEVICEINFO
{
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

#if defined(HAS_KINETISK_UART5)
// Instance for UART0, UART1, UART2, UART3, UART4, UART5
static TeensyDmx *uartInstances[6] = {0};
#elif defined(HAS_KINETISK_UART4)
// Instance for UART0, UART1, UART2, UART3, UART4
static TeensyDmx *uartInstances[5] = {0};
#elif defined(HAS_KINETISK_UART3)
// Instance for UART0, UART1, UART2, UART3
static TeensyDmx *uartInstances[4] = {0};
#else
// Instance for UART0, UART1, UART2
static TeensyDmx *uartInstances[3] = {0};
#endif

static inline void putInt(void* const buffer, const size_t offset, const uint16_t value)
{
    reinterpret_cast<byte*>(buffer)[offset] = value >> 8;
    reinterpret_cast<byte*>(buffer)[offset + 1] = value & 0xff;
}

TeensyDmx::TeensyDmx(HardwareSerial& uart) :
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
    m_redePin(nullptr),
    m_rdmMute(false),
    m_identifyMode(false),
    m_rdm(nullptr),
    m_deviceLabel{0}
{
    if (&m_uart == &Serial1) {
        uartInstances[0] = this;
    } else if (&m_uart == &Serial2) {
        uartInstances[1] = this;
    } else if (&m_uart == &Serial3) {
        uartInstances[2] = this;
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        uartInstances[3] = this;
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        uartInstances[4] = this;
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        uartInstances[5] = this;
    }
#endif
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, uint8_t redePin) :
    TeensyDmx(uart)
{
    pinMode(redePin, OUTPUT);
    m_redePin = portOutputRegister(redePin);
    *m_redePin = 0;
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm) :
    TeensyDmx(uart)
{
    m_rdm = rdm;
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm, uint8_t redePin) :
    TeensyDmx(uart, redePin)
{
    m_rdm = rdm;
}

const volatile uint8_t* TeensyDmx::getBuffer() const
{
    return m_inactiveBuffer;
}

bool TeensyDmx::isIdentify() const
{
    return m_identifyMode;
}

const char* TeensyDmx::getLabel() const
{
    return m_deviceLabel;
}

void TeensyDmx::setMode(TeensyDmx::Mode mode)
{
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
            if (m_redePin != nullptr) {
                *m_redePin = 0;  // Off puts in receive state so as to be passive
            }
            break;
    }
}

void TeensyDmx::setChannel(const uint16_t address, const uint8_t value)
{
    if (address < DMX_BUFFER_SIZE) {
        m_activeBuffer[address] = value;
    }
}

void TeensyDmx::setChannels(
        const uint16_t startAddress,
        const uint8_t* values,
        const uint16_t length)
{
    uint16_t currentAddress = 0;
    while (currentAddress < startAddress && currentAddress < DMX_BUFFER_SIZE) {
        m_activeBuffer[currentAddress] = 0;
        ++currentAddress;
    }
    for (uint16_t i = 0; i < length && currentAddress < DMX_BUFFER_SIZE; ++i) {
        m_activeBuffer[currentAddress] = values[i];
        ++currentAddress;
    }
    while (currentAddress < DMX_BUFFER_SIZE) {
        m_activeBuffer[currentAddress] = 0;
        ++currentAddress;
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
            m_dmxBufferIndex = 0;
        } else {
            m_uart.write(m_activeBuffer[m_dmxBufferIndex]);
            ++m_dmxBufferIndex;
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

#ifdef HAS_KINETISK_UART3
void uart3_status_isr();  // Back reference to serial4.c
void UART3TxStatus()
{
    if ((UART3_C2 & UART_C2_TCIE) && (UART3_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[3]->nextTx();
    }
    // Call standard ISR too
    uart3_status_isr();
}
#endif

#ifdef HAS_KINETISK_UART4
void uart4_status_isr();  // Back reference to serial5.c
void UART4TxStatus()
{
    if ((UART4_C2 & UART_C2_TCIE) && (UART4_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[4]->nextTx();
    }
    // Call standard ISR too
    uart4_status_isr();
}
#endif

#ifdef HAS_KINETISK_UART5
void uart5_status_isr();  // Back reference to serial6.c
void UART5TxStatus()
{
    if ((UART5_C2 & UART_C2_TCIE) && (UART5_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[5]->nextTx();
    }
    // Call standard ISR too
    uart5_status_isr();
}
#endif

void TeensyDmx::startTransmit()
{
    if (m_redePin != nullptr) {
        *m_redePin = 1;
    }

    m_dmxBufferIndex = 0;

    if (&m_uart == &Serial1) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART0_STATUS, UART0TxStatus);
    } else if (&m_uart == &Serial2) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART1_STATUS, UART1TxStatus);
    } else if (&m_uart == &Serial3) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART2_STATUS, UART2TxStatus);
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART3_STATUS, UART3TxStatus);
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART4_STATUS, UART4TxStatus);
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        // Change interrupt vector to mine to monitor TX complete
        attachInterruptVector(IRQ_UART5_STATUS, UART5TxStatus);
    }
#endif

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
    } else if (&m_uart == &Serial2) {
        attachInterruptVector(IRQ_UART1_STATUS, uart1_status_isr);
    } else if (&m_uart == &Serial3) {
        attachInterruptVector(IRQ_UART2_STATUS, uart2_status_isr);
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        attachInterruptVector(IRQ_UART3_STATUS, uart3_status_isr);
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        attachInterruptVector(IRQ_UART4_STATUS, uart4_status_isr);
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        attachInterruptVector(IRQ_UART5_STATUS, uart5_status_isr);
    }
#endif
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

void TeensyDmx::rdmUniqueBranch(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (m_rdmMute) return;

    if (rdm->Length != 36) return;
    if (rdm->DataLength != 12) return;

    if (memcmp(rdm->Data, _devID, sizeof(_devID)) <= 0 &&
            memcmp(_devID, rdm->Data+6, sizeof(_devID)) <= 0) {
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
        if (m_redePin != nullptr) {
            *m_redePin = 1;
        }
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
}

void TeensyDmx::rdmUnmute(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength == 0) {
        m_rdmMute = false;
        // Control field
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->DataLength = 2;
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmMute(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength == 0) {
        m_rdmMute = true;
        // Control field
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->DataLength = 2;
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmSetIdentify(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength != 1) {
        // Oversized data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if ((rdm->Data[0] != 0) && (rdm->Data[0] != 1)) {
        // Out of range data
        respondMessage(timingStart, E120_NR_DATA_OUT_OF_RANGE);
    } else {
        m_identifyMode = rdm->Data[0] != 0;
        m_rdmChange = true;
        rdm->DataLength = 0;
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmSetDeviceLabel(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength > sizeof(m_deviceLabel)) {
        // Oversized data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else {
        memcpy(m_deviceLabel, rdm->Data, rdm->DataLength);
        m_deviceLabel[rdm->DataLength] = '\0';
        rdm->DataLength = 0;
        m_rdmChange = true;
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmSetStartAddress(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength != 2) {
        // Oversized data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else {
        uint16_t newStartAddress = (rdm->Data[0] << 8) | (rdm->Data[1]);
        if ((newStartAddress <= 0) || (newStartAddress > DMX_BUFFER_SIZE)) {
            // Out of range start address
            respondMessage(timingStart, E120_NR_DATA_OUT_OF_RANGE);
        } else if (m_rdm == nullptr) {
            respondMessage(timingStart, E120_NR_HARDWARE_FAULT);
        } else {
            m_rdm->startAddress = newStartAddress;
            rdm->DataLength = 0;
            m_rdmChange = true;
            respondMessage(timingStart, NACK_WAS_ACK);
        }
    }
}

void TeensyDmx::rdmSetParameters(const unsigned long timingStart, struct RDMDATA* rdm)
{
    respondMessage(timingStart, E120_NR_UNSUPPORTED_COMMAND_CLASS);
}

void TeensyDmx::rdmGetIdentify(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else {
        rdm->Data[0] = m_identifyMode;
        rdm->DataLength = 1;
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetDeviceInfo(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else {
        // return all device info data
        // The data has to be responsed in the Data buffer.
        DEVICEINFO *devInfo = (DEVICEINFO *)(rdm->Data);

        devInfo->protocolMajor = 1;
        devInfo->protocolMinor = 0;
        putInt(&devInfo->productCategory, 0, E120_PRODUCT_CATEGORY_DIMMER_CS_LED);
        devInfo->softwareVersion = 0x00000010;  // Endian swapped
        devInfo->currentPersonality = 1;
        devInfo->personalityCount = 1;
        devInfo->subDeviceCount = 0;
        devInfo->sensorCount = 0;
        if (m_rdm == nullptr) {
            devInfo->deviceModel = 0;
            devInfo->startAddress = 0;
            devInfo->footprint = 0;
        } else {
            putInt(&devInfo->deviceModel, 0, m_rdm->deviceModelId);
            putInt(&devInfo->startAddress, 0, m_rdm->startAddress);
            putInt(&devInfo->footprint, 0, m_rdm->footprint);
        }

        rdm->DataLength = sizeof(DEVICEINFO);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetManufacturerLabel(
        const unsigned long timingStart,
        struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else if (m_rdm == nullptr) {
        rdm->DataLength = 0;
        respondMessage(timingStart, NACK_WAS_ACK);
    } else {
        // return the manufacturer label
        rdm->DataLength = strlen(m_rdm->manufacturerLabel);
        memcpy(rdm->Data, m_rdm->manufacturerLabel, rdm->DataLength);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetModelDescription(
        const unsigned long timingStart,
        struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else if (m_rdm == nullptr) {
        rdm->DataLength = 0;
        respondMessage(timingStart, NACK_WAS_ACK);
    } else {
        // return the DEVICE MODEL DESCRIPTION
        rdm->DataLength = strlen(m_rdm->deviceModel);
        memcpy(rdm->Data, m_rdm->deviceModel, rdm->DataLength);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetDeviceLabel(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else {
        rdm->DataLength = strlen(m_deviceLabel);
        memcpy(rdm->Data, m_deviceLabel, rdm->DataLength);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetSoftwareVersion(
        const unsigned long timingStart,
        struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else if (m_rdm == nullptr) {
        rdm->DataLength = 0;
        respondMessage(timingStart, NACK_WAS_ACK);
    } else {
        // return the SOFTWARE_VERSION_LABEL
        rdm->DataLength = strlen(m_rdm->softwareLabel);
        memcpy(rdm->Data, m_rdm->softwareLabel, rdm->DataLength);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::rdmGetStartAddress(const unsigned long timingStart, struct RDMDATA* rdm)
{
   if (rdm->DataLength > 0) {
       // Unexpected data
       respondMessage(timingStart, E120_NR_FORMAT_ERROR);
   } else if (rdm->SubDev != 0) {
       // No sub-devices supported
       respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
   } else {
       if (m_rdm == nullptr) {
           putInt(rdm->Data, 0, 0);
       } else {
           putInt(rdm->Data, 0, m_rdm->startAddress);
       }
       rdm->DataLength = 2;
       respondMessage(timingStart, NACK_WAS_ACK);
   }
}

void TeensyDmx::rdmGetParameters(const unsigned long timingStart, struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        respondMessage(timingStart, E120_NR_FORMAT_ERROR);
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        respondMessage(timingStart, E120_NR_SUB_DEVICE_OUT_OF_RANGE);
    } else {
        if (m_rdm == nullptr) {
            rdm->DataLength = 6;
        } else {
            rdm->DataLength = 2 * (3 + m_rdm->additionalCommandsLength);
            for (int n = 0; n < m_rdm->additionalCommandsLength; ++n) {
                putInt(rdm->Data, 6+n+n, m_rdm->additionalCommands[n]);
            }
        }
        putInt(rdm->Data, 0, E120_MANUFACTURER_LABEL);
        putInt(rdm->Data, 2, E120_DEVICE_MODEL_DESCRIPTION);
        putInt(rdm->Data, 4, E120_DEVICE_LABEL);
        respondMessage(timingStart, NACK_WAS_ACK);
    }
}

void TeensyDmx::processRDM()
{
    unsigned long timingStart = micros();
    struct RDMDATA* rdm = (struct RDMDATA*)(m_activeBuffer);

    bool isForMe = (memcmp(rdm->DestID, _devID, sizeof(_devID)) == 0);
    if (isForMe ||
            memcmp(rdm->DestID, _devIDAll, sizeof(_devIDAll)) == 0 ||
            memcmp(rdm->DestID, _devIDGroup, sizeof(_devIDGroup)) == 0) {

        uint16_t parameter = (rdm->Parameter << 8) | ((rdm->Parameter >> 8) & 0xff);
        switch (rdm->CmdClass)
        {
            case E120_DISCOVERY_COMMAND:
                switch (parameter)
                {
                    case E120_DISC_UNIQUE_BRANCH:
                        rdmUniqueBranch(timingStart, rdm);
                        break;
                    case E120_DISC_UN_MUTE:
                        if (isForMe)
                        {
                            rdmUnmute(timingStart, rdm);
                        }
                        break;
                    case E120_DISC_MUTE:
                        if (isForMe)
                        {
                            rdmMute(timingStart, rdm);
                        }
                        break;
                    default:
                        respondMessage(timingStart, E120_NR_UNKNOWN_PID);
                        break;
                }
                break;
            case E120_SET_COMMAND:
                switch (parameter)
                {
                    case E120_IDENTIFY_DEVICE:
                        rdmSetIdentify(timingStart, rdm);
                        break;
                    case E120_DEVICE_LABEL:
                        rdmSetDeviceLabel(timingStart, rdm);
                        break;
                    case E120_DMX_START_ADDRESS:
                        rdmSetStartAddress(timingStart, rdm);
                        break;
                    case E120_SUPPORTED_PARAMETERS:
                        rdmSetParameters(timingStart, rdm);
                        break;
                    default:
                        respondMessage(timingStart, E120_NR_UNKNOWN_PID);
                        break;
                }
                break;
            case E120_GET_COMMAND:
                switch (parameter)
                {
                    case E120_IDENTIFY_DEVICE:
                        rdmGetIdentify(timingStart, rdm);
                        break;
                    case E120_DEVICE_INFO:
                        rdmGetDeviceInfo(timingStart, rdm);
                        break;
                    case E120_MANUFACTURER_LABEL:
                        rdmGetManufacturerLabel(timingStart, rdm);
                        break;
                    case E120_DEVICE_MODEL_DESCRIPTION:
                        rdmGetModelDescription(timingStart, rdm);
                        break;
                    case E120_DEVICE_LABEL:
                        rdmGetDeviceLabel(timingStart, rdm);
                        break;
                    case E120_SOFTWARE_VERSION_LABEL:
                        rdmGetSoftwareVersion(timingStart, rdm);
                        break;
                    case E120_DMX_START_ADDRESS:
                        rdmGetStartAddress(timingStart, rdm);
                        break;
                    case E120_SUPPORTED_PARAMETERS:
                        rdmGetParameters(timingStart, rdm);
                        break;
                    default:
                        respondMessage(timingStart, E120_NR_UNKNOWN_PID);
                        break;
                }
                break;
            default:
                respondMessage(timingStart, E120_NR_UNKNOWN_PID);
                break;
        }
    }
}

void TeensyDmx::respondMessage(unsigned long timingStart, uint16_t nackReason)
{
    uint16_t i;
    uint16_t checkSum = 0;
    struct RDMDATA* rdm = (struct RDMDATA*)(m_activeBuffer);

    // TIMING: don't send too fast, min: 176 microseconds
    timingStart = micros() - timingStart;
    if (timingStart < 176) {
        delayMicroseconds(176 - timingStart);
    }

    // no need to set these data fields:
    // StartCode, SubStartCode
    rdm->MessageCount = 0; // Number of queued messages
    if (nackReason == NACK_WAS_ACK) {
        rdm->ResponseType = E120_RESPONSE_TYPE_ACK;
    } else {
        rdm->ResponseType = E120_RESPONSE_TYPE_NACK_REASON;
        rdm->DataLength = 2;
        rdm->Data[0] = (nackReason >> 8) & 0xFF;
        rdm->Data[1] = nackReason & 0xFF;
    }
    rdm->Length = rdm->DataLength + 24; // total packet length

    // swap SrcID into DestID for sending back.
    memcpy(rdm->DestID, rdm->SourceID, sizeof(rdm->SourceID));
    memcpy(rdm->SourceID, _devID, sizeof(_devID));

    ++(rdm->CmdClass);
    // Parameter

    // prepare buffer and Checksum
    checkSum += E120_SC_RDM;
    for (i = 0; i < rdm->Length; i++) {
        checkSum += m_activeBuffer[i];
    }

    // Send reply
    stopReceive();
    if (m_redePin != nullptr) {
        *m_redePin = 1;
    }
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

#ifdef HAS_KINETISK_UART3
void uart3_error_isr();  // Back reference to serial4.c
// UART3 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART3RxError(void)
{
    // On break, uart3_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART3_S1 & UART_S1_FE)
        (void) UART3_D;

    uartInstances[3]->completeFrame();
}
#endif

#ifdef HAS_KINETISK_UART4
void uart4_error_isr();  // Back reference to serial5.c
// UART4 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART4RxError(void)
{
    // On break, uart4_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART4_S1 & UART_S1_FE)
        (void) UART4_D;

    uartInstances[4]->completeFrame();
}
#endif

#ifdef HAS_KINETISK_UART5
void uart5_error_isr();  // Back reference to serial6.c
// UART2 will throw a frame error on the DMX break pulse.  That's our
// cue to switch buffers and reset the index to zero
void UART5RxError(void)
{
    // On break, uart5_status_isr() will probably have already
    // fired and read the data buffer, clearing the framing error.
    // If for some reason it hasn't, make sure we consume the 0x00
    // byte that was received.
    if (UART5_S1 & UART_S1_FE)
        (void) UART5_D;

    uartInstances[5]->completeFrame();
}
#endif

#ifndef IRQ_UART0_ERROR
void UART0RxStatus()
{
    if (UART0_S1 & UART_S1_FE)
    {
        (void) UART0_D;
        uart0_status_isr();
        uartInstances[0]->completeFrame();
    }
    else
    {
        uart0_status_isr();
    }
}

void UART1RxStatus()
{
    if (UART1_S1 & UART_S1_FE)
    {
        (void) UART1_D;
        uart1_status_isr();
        uartInstances[1]->completeFrame();
    }
    else
    {
        uart1_status_isr();
    }
}

void UART2RxStatus()
{
    if (UART2_S1 & UART_S1_FE)
    {
        (void) UART2_D;
        uart2_status_isr();
        uartInstances[2]->completeFrame();
    }
    else
    {
        uart2_status_isr();
    }
}
#endif  // IRQ_UART0_ERROR

void TeensyDmx::startReceive()
{
    if (m_redePin != nullptr) {
        *m_redePin = 0;
    }

    // UART Initialisation
    m_uart.begin(250000);

#ifndef IRQ_UART0_ERROR
    if (&m_uart == &Serial1) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART0 receive interrupt immediately after each byte received
        UART0_RWFIFO = 1;

        // Enable UART0 interrupt on frame error and enable IRQ
        UART0_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);

        attachInterruptVector(IRQ_UART0_STATUS, UART0RxStatus);
    } else if (&m_uart == &Serial2) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART1 receive interrupt immediately after each byte received
        UART1_RWFIFO = 1;

        // Enable UART0 interrupt on frame error and enable IRQ
        UART1_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);

        attachInterruptVector(IRQ_UART1_STATUS, UART1RxStatus);
    } else if (&m_uart == &Serial3) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART2 receive interrupt immediately after each byte received
        UART2_RWFIFO = 1;

        // Enable UART2 interrupt on frame error and enable IRQ
        UART2_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);

        attachInterruptVector(IRQ_UART2_STATUS, UART2RxStatus);
    }
    // TODO: Does this need to handle Serial4-6 here too?
#else
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

        attachInterruptVector(IRQ_UART0_ERROR, UART0RxError);
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

        attachInterruptVector(IRQ_UART1_ERROR, UART1RxError);
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

        attachInterruptVector(IRQ_UART2_ERROR, UART2RxError);
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        // Fire UART3 receive interrupt immediately after each byte received
        UART3_RWFIFO = 1;

        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART3_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART3_STATUS) + 1);

        // Enable UART2 interrupt on frame error and enable IRQ
        UART3_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);

        attachInterruptVector(IRQ_UART3_ERROR, UART3RxError);
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        // Fire UART4 receive interrupt immediately after each byte received
        UART4_RWFIFO = 1;

        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART4_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART4_STATUS) + 1);

        // Enable UART2 interrupt on frame error and enable IRQ
        UART4_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);

        attachInterruptVector(IRQ_UART4_ERROR, UART4RxError);
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        // Fire UART5 receive interrupt immediately after each byte received
        UART5_RWFIFO = 1;

        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART5_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART5_STATUS) + 1);

        // Enable UART2 interrupt on frame error and enable IRQ
        UART5_C3 |= UART_C3_FEIE;
        NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);

        attachInterruptVector(IRQ_UART5_ERROR, UART5RxError);
    }
#endif
#endif

    m_dmxBufferIndex = 0;
    m_state = State::IDLE;
}

void TeensyDmx::stopReceive()
{
    m_uart.end();

#ifndef IRQ_UART0_ERROR
    if (&m_uart == &Serial1) {
        UART0_RWFIFO = 0;
        UART0_C3 &= ~UART_C3_FEIE;
        attachInterruptVector(IRQ_UART0_STATUS, uart0_status_isr);
    } else if (&m_uart == &Serial2) {
        UART1_RWFIFO = 0;
        UART1_C3 &= ~UART_C3_FEIE;
        attachInterruptVector(IRQ_UART1_STATUS, uart1_status_isr);
    } else if (&m_uart == &Serial3) {
        UART2_RWFIFO = 0;
        UART2_C3 &= ~UART_C3_FEIE;
        attachInterruptVector(IRQ_UART2_STATUS, uart2_status_isr);
    }
#else
    if (&m_uart == &Serial1) {
        UART0_RWFIFO = 0;
        UART0_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
        attachInterruptVector(IRQ_UART0_ERROR, uart0_error_isr);
    } else if (&m_uart == &Serial2) {
        UART1_RWFIFO = 0;
        UART1_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
        attachInterruptVector(IRQ_UART1_ERROR, uart1_error_isr);
    } else if (&m_uart == &Serial3) {
        UART2_RWFIFO = 0;
        UART2_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
        attachInterruptVector(IRQ_UART2_ERROR, uart2_error_isr);
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        UART3_RWFIFO = 0;
        UART3_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
        attachInterruptVector(IRQ_UART3_ERROR, uart3_error_isr);
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        UART4_RWFIFO = 0;
        UART4_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
        attachInterruptVector(IRQ_UART4_ERROR, uart4_error_isr);
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        UART5_RWFIFO = 0;
        UART5_C3 &= ~UART_C3_FEIE;
        NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
        attachInterruptVector(IRQ_UART5_ERROR, uart5_error_isr);
    }
#endif
#endif
}

void TeensyDmx::readBytes()
{
    __disable_irq();  // Prevents conflicts with the error ISR

    int available = m_uart.available();
    while (available--)
    {
        switch (m_state)
        {
            case State::BREAK:
                switch (m_uart.read())
                {
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
                ++m_dmxBufferIndex;
                m_activeBuffer[m_dmxBufferIndex] = m_uart.read();

                if (m_dmxBufferIndex >= DMX_BUFFER_SIZE) {
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
