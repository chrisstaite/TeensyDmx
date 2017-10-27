#include "TeensyDmx.h"
#include "rdm.h"

static constexpr uint32_t BREAKSPEED = 100000;
static constexpr uint32_t RDM_BREAKSPEED = 45500;
static constexpr uint32_t BREAKFORMAT = SERIAL_8E1;
static constexpr uint32_t DMXSPEED = 250000;
static constexpr uint32_t DMXFORMAT = SERIAL_8N2;
static constexpr uint16_t NACK_WAS_ACK = 0xffff;  // Send an ACK, not a NACK

// The Device ID for addressing all devices: 6 times 0xFF.
static constexpr byte _devIDAll[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// The DEVICE_INFO_GET_RESPONSE structure (length = 19) has to be responsed for E120_DEVICE_INFO
// See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DEVICE_INFO_GET_RESPONSE
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
} __attribute__((__packed__)); // struct DEVICE_INFO_GET_RESPONSE
static_assert((sizeof(DEVICE_INFO_GET_RESPONSE)==19), "Invalid size for DEVICE_INFO_GET_RESPONSE struct, is it packed?");

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

static inline void putUInt16(void* const buffer, const size_t offset, const uint16_t value)
{
    reinterpret_cast<byte*>(buffer)[offset] = value >> 8;
    reinterpret_cast<byte*>(buffer)[offset + 1] = value & 0xff;
}

static inline void putUInt32(void* const buffer, const size_t offset, const uint32_t value)
{
    reinterpret_cast<byte*>(buffer)[offset] = (value & 0xff000000) >> 24;
    reinterpret_cast<byte*>(buffer)[offset + 1] = (value & 0x00ff0000) >> 16;
    reinterpret_cast<byte*>(buffer)[offset + 2] = (value & 0x0000ff00) >> 8;
    reinterpret_cast<byte*>(buffer)[offset + 3] = (value & 0x000000ff);
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm, uint8_t redePin) :
    TeensyDmx(uart, rdm)
{
    pinMode(redePin, OUTPUT);
    m_redePin = portOutputRegister(redePin);
    *m_redePin = 0;
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm) :
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

    if (rdm != nullptr) {
        m_rdm = rdm;
        m_vendorcastUid[0] = m_rdm->uid[0];
        m_vendorcastUid[1] = m_rdm->uid[1];
        memset(&m_vendorcastUid[2], 0xff, 4);  // Set the rest of the bytes to 0xff
    } else {
        m_rdm = nullptr;
    }
}

const volatile uint8_t* TeensyDmx::getBuffer() const
{
    if (m_mode == DMX_IN) {
        // DMX Rx is double buffered due to the interupt handler
        return m_inactiveBuffer;
    } else {
        return m_activeBuffer;
    }
}

uint8_t TeensyDmx::getChannel(const uint16_t address)
{
    if (address < DMX_BUFFER_SIZE) {
        return getBuffer()[address];
    } else {
        return 0;
    }
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
    // Stop what we were doing
    m_state = IDLE;

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

    switch (m_state)
    {
        case State::DMX_RECV:
        case State::DMX_COMPLETE:
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
            break;
        case State::RDM_COMPLETE:
            // Check if we need to reply to this RDM message
            m_state = State::IDLE; // Stop the ISR messing up things
            processRDM();
            break;
        case State::RDM_RECV:
        case State::RDM_RECV_CHECKSUM_HI:
        case State::RDM_RECV_CHECKSUM_LO:
            // Partial RDM packet then break
            m_state = State::IDLE; // Give up and start over
            break;
        default:
            // Unknown, ASC? frame
            break;
    }
    m_dmxBufferIndex = 0;
    m_state = State::BREAK;
}

void TeensyDmx::rdmDiscUniqueBranch(struct RDMDATA* rdm)
{
    if (m_rdm == nullptr) return;

    if (m_rdmMute) return;

    if (rdm->Length != (RDM_PACKET_SIZE_NO_PD + sizeof(DISC_UNIQUE_BRANCH_REQUEST))) return;
    if (rdm->DataLength != sizeof(DISC_UNIQUE_BRANCH_REQUEST)) return;

    DISC_UNIQUE_BRANCH_REQUEST *dub_request = (DISC_UNIQUE_BRANCH_REQUEST *)(rdm->Data);

    if (memcmp(dub_request->lowerBoundUID, m_rdm->uid, RDM_UID_LENGTH) <= 0 &&
            memcmp(m_rdm->uid, dub_request->upperBoundUID, RDM_UID_LENGTH) <= 0) {
        // I'm in range - say hello to the lovely controller

        // respond with the special discovery message !
        struct DISC_UNIQUE_BRANCH_RESPONSE *dub_response = (struct DISC_UNIQUE_BRANCH_RESPONSE*)(&m_rdmBuffer.discovery);

        // fill in the discovery response structure
        for (byte i = 0; i < 7; ++i) {
            dub_response->headerFE[i] = 0xFE;
        }
        dub_response->headerAA = 0xAA;
        for (byte i = 0; i < 6; ++i) {
            dub_response->maskedDevID[i+i]   = m_rdm->uid[i] | 0xAA;
            dub_response->maskedDevID[i+i+1] = m_rdm->uid[i] | 0x55;
        }

        uint16_t checksum = rdmCalculateChecksum(dub_response->maskedDevID, sizeof(dub_response->maskedDevID));

        dub_response->checksum[0] = (checksum >> 8)   | 0xAA;
        dub_response->checksum[1] = (checksum >> 8)   | 0x55;
        dub_response->checksum[2] = (checksum & 0xFF) | 0xAA;
        dub_response->checksum[3] = (checksum & 0xFF) | 0x55;

        // Send reply
        stopReceive();
        if (m_redePin != nullptr) {
            *m_redePin = 1;
        }
        m_dmxBufferIndex = 0;
        // No break for DUB
        m_uart.begin(DMXSPEED, DMXFORMAT);
        for (uint16_t i = 0; i < sizeof(DISC_UNIQUE_BRANCH_RESPONSE); ++i) {
            m_uart.write(m_rdmBuffer.buffer[i]);
            m_uart.flush();
        }
        startReceive();
    }
}

uint16_t TeensyDmx::rdmDiscUnMute(struct RDMDATA* rdm)
{
    if (rdm->DataLength == 0) {
        m_rdmMute = false;
        // Control field
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->DataLength = 2;
        return NACK_WAS_ACK;
    } else {
        return E120_NR_FORMAT_ERROR;
    }
}

uint16_t TeensyDmx::rdmDiscMute(struct RDMDATA* rdm)
{
    if (rdm->DataLength == 0) {
        m_rdmMute = true;
        // Control field
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->DataLength = 2;
        return NACK_WAS_ACK;
    } else {
        return E120_NR_FORMAT_ERROR;
    }
}

uint16_t TeensyDmx::rdmSetIdentifyDevice(struct RDMDATA* rdm)
{
    if (rdm->DataLength != 1) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    } else if ((rdm->Data[0] != 0) && (rdm->Data[0] != 1)) {
        // Out of range data
        return E120_NR_DATA_OUT_OF_RANGE;
    } else {
        m_identifyMode = rdm->Data[0] != 0;
        m_rdmChange = true;
        rdm->DataLength = 0;
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmSetDeviceLabel(struct RDMDATA* rdm)
{
    if (rdm->DataLength > RDM_MAX_STRING_LENGTH) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    } else {
        memcpy(m_deviceLabel, rdm->Data, rdm->DataLength);
        m_deviceLabel[rdm->DataLength] = '\0';
        rdm->DataLength = 0;
        m_rdmChange = true;
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmSetDMXStartAddress(struct RDMDATA* rdm)
{
    if (rdm->DataLength != 2) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    } else {
        uint16_t newStartAddress = READINT(rdm->Data);
        if ((newStartAddress <= 0) || (newStartAddress > DMX_BUFFER_SIZE)) {
            // Out of range start address
            return E120_NR_DATA_OUT_OF_RANGE;
        } else if (m_rdm == nullptr) {
            return E120_NR_HARDWARE_FAULT;
        } else {
            m_rdm->startAddress = newStartAddress;
            rdm->DataLength = 0;
            m_rdmChange = true;
            return NACK_WAS_ACK;
        }
    }
}

uint16_t TeensyDmx::rdmGetIdentifyDevice(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        rdm->Data[0] = m_identifyMode;
        rdm->DataLength = 1;
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDeviceInfo(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        // return all device info data
        // The data has to be responsed in the Data buffer.
        DEVICE_INFO_GET_RESPONSE *devInfo = (DEVICE_INFO_GET_RESPONSE *)(rdm->Data);

        devInfo->protocolMajor = 1;
        devInfo->protocolMinor = 0;
        devInfo->currentPersonality = 1;
        devInfo->personalityCount = 1;
        devInfo->subDeviceCount = 0;
        devInfo->sensorCount = 0;
        if (m_rdm == nullptr) {
            devInfo->deviceModel = 0;
            putUInt16(&devInfo->productCategory, 0, E120_PRODUCT_CATEGORY_NOT_DECLARED);
            devInfo->softwareVersion = 0;
            devInfo->startAddress = 0;
            devInfo->footprint = 0;
        } else {
            putUInt16(&devInfo->deviceModel, 0, m_rdm->deviceModelId);
            putUInt16(&devInfo->productCategory, 0, m_rdm->productCategory);
            putUInt32(&devInfo->softwareVersion, 0, m_rdm->softwareVersionId);
            putUInt16(&devInfo->startAddress, 0, m_rdm->startAddress);
            putUInt16(&devInfo->footprint, 0, m_rdm->footprint);
        }

        rdm->DataLength = sizeof(DEVICE_INFO_GET_RESPONSE);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetManufacturerLabel(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the manufacturer label
        rdm->DataLength = strnlen(m_rdm->manufacturerLabel, RDM_MAX_STRING_LENGTH);
        memcpy(rdm->Data, m_rdm->manufacturerLabel, rdm->DataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDeviceModelDescription(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the DEVICE MODEL DESCRIPTION
        rdm->DataLength = strnlen(m_rdm->deviceModel, RDM_MAX_STRING_LENGTH);
        memcpy(rdm->Data, m_rdm->deviceModel, rdm->DataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDeviceLabel(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        rdm->DataLength = strnlen(m_deviceLabel, RDM_MAX_STRING_LENGTH);
        memcpy(rdm->Data, m_deviceLabel, rdm->DataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetSoftwareVersionLabel(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the SOFTWARE_VERSION_LABEL
        rdm->DataLength = strnlen(m_rdm->softwareLabel, RDM_MAX_STRING_LENGTH);
        memcpy(rdm->Data, m_rdm->softwareLabel, rdm->DataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDMXStartAddress(struct RDMDATA* rdm)
{
   if (rdm->DataLength > 0) {
       // Unexpected data
       return E120_NR_FORMAT_ERROR;
   } else if (rdm->SubDev != 0) {
       // No sub-devices supported
       return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
   } else {
       if (m_rdm == nullptr) {
           putUInt16(rdm->Data, 0, 0);
       } else {
           putUInt16(rdm->Data, 0, m_rdm->startAddress);
       }
       rdm->DataLength = sizeof(m_rdm->startAddress);
       return NACK_WAS_ACK;
   }
}

uint16_t TeensyDmx::rdmGetSupportedParameters(struct RDMDATA* rdm)
{
    if (rdm->DataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        if (m_rdm == nullptr) {
            rdm->DataLength = 6;
        } else {
            rdm->DataLength = 2 * (3 + m_rdm->additionalCommandsLength);
            for (int n = 0; n < m_rdm->additionalCommandsLength; ++n) {
                putUInt16(rdm->Data, 6+n+n, m_rdm->additionalCommands[n]);
            }
        }
        putUInt16(rdm->Data, 0, E120_MANUFACTURER_LABEL);
        putUInt16(rdm->Data, 2, E120_DEVICE_MODEL_DESCRIPTION);
        putUInt16(rdm->Data, 4, E120_DEVICE_LABEL);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmCalculateChecksum(uint8_t* data, uint8_t length)
{
    uint16_t checksum = 0;

    // calculate checksum
    for (unsigned int i = 0; i < length; ++i) {
        checksum += *data;
        ++data;
    }

    return checksum;
}

void TeensyDmx::processRDM()
{
    uint16_t nackReason = E120_NR_UNKNOWN_PID;

    m_state = IDLE;
    unsigned long timingStart = micros();
    struct RDMDATA* rdm = (struct RDMDATA*)(&m_rdmBuffer.packet);

    if (m_rdm != nullptr) {
        bool isForMe = (memcmp(rdm->DestID, m_rdm->uid, RDM_UID_LENGTH) == 0);
        if (isForMe ||
                memcmp(rdm->DestID, _devIDAll, RDM_UID_LENGTH) == 0 ||
                memcmp(rdm->DestID, m_vendorcastUid, RDM_UID_LENGTH) == 0) {

            bool sendResponse = true;

            uint16_t parameter = SWAPINT(rdm->Parameter);
            if (rdm->CmdClass == E120_DISCOVERY_COMMAND) {
                switch (parameter) {
                    case E120_DISC_UNIQUE_BRANCH:
                        rdmDiscUniqueBranch(rdm);
                        sendResponse = false;  // DUB is special
                        break;
                    case E120_DISC_UN_MUTE:
                        nackReason = rdmDiscUnMute(rdm);
                        break;
                    case E120_DISC_MUTE:
                        nackReason = rdmDiscMute(rdm);
                        break;
                    default:
                        // Don't respond, unknown DISCOVERY PID
                        sendResponse = false;
                        break;
                }
                if (nackReason != NACK_WAS_ACK) {
                        // Only send ACKs for DISCOVERY, don't NACK
                        sendResponse = false;
                }
            } else {
                if ((rdm->CmdClass == E120_GET_COMMAND) || (rdm->CmdClass == E120_SET_COMMAND)) {
                    switch (parameter) {
                        case E120_IDENTIFY_DEVICE:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = rdmSetIdentifyDevice(rdm);
                            } else {
                                nackReason = rdmGetIdentifyDevice(rdm);
                            }
                            break;
                        case E120_DEVICE_LABEL:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = rdmSetDeviceLabel(rdm);
                            } else {
                                nackReason = rdmGetDeviceLabel(rdm);
                            }
                            break;
                        case E120_DMX_START_ADDRESS:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = rdmSetDMXStartAddress(rdm);
                            } else {
                                nackReason = rdmGetDMXStartAddress(rdm);
                            }
                            break;
                        case E120_SUPPORTED_PARAMETERS:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                            } else {
                                nackReason = rdmGetSupportedParameters(rdm);
                            }
                            break;
                        case E120_DEVICE_INFO:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                            } else {
                                nackReason = rdmGetDeviceInfo(rdm);
                            }
                            break;
                        case E120_MANUFACTURER_LABEL:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                            } else {
                                nackReason = rdmGetManufacturerLabel(rdm);
                            }
                            break;
                        case E120_DEVICE_MODEL_DESCRIPTION:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                            } else {
                                nackReason = rdmGetDeviceModelDescription(rdm);
                            }
                            break;
                        case E120_SOFTWARE_VERSION_LABEL:
                            if (rdm->CmdClass == E120_SET_COMMAND) {
                                nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                            } else {
                                nackReason = rdmGetSoftwareVersionLabel(rdm);
                            }
                            break;
                        default:
                            nackReason = E120_NR_UNKNOWN_PID;
                            break;
                    }
                } else {
                    // Unknown command class
                    nackReason = E120_NR_FORMAT_ERROR;
                }
            }
            if (isForMe && sendResponse) {
                respondMessage(timingStart, nackReason);
            }
        } else {
            // Not for me
        }
    }
}

void TeensyDmx::respondMessage(unsigned long timingStart, uint16_t nackReason)
{

    struct RDMDATA* rdm = (struct RDMDATA*)(&m_rdmBuffer.packet);

    // TIMING: don't send too fast, min: 176 microseconds
    timingStart = micros() - timingStart;
    if (timingStart < 176) {
        delayMicroseconds(176 - timingStart);
    }

    // swap SrcID into DestID for sending back.
    memcpy(rdm->DestID, rdm->SourceID, RDM_UID_LENGTH);
    if (m_rdm != nullptr) {
        memcpy(rdm->SourceID, m_rdm->uid, RDM_UID_LENGTH);
    } else {
        nackReason = E120_NR_HARDWARE_FAULT;
    }


    // no need to set these data fields:
    // StartCode, SubStartCode
    rdm->MessageCount = 0; // Number of queued messages
    if (nackReason == NACK_WAS_ACK) {
        rdm->ResponseType = E120_RESPONSE_TYPE_ACK;
    } else {
        rdm->ResponseType = E120_RESPONSE_TYPE_NACK_REASON;
        rdm->DataLength = 2;
        putUInt16(&rdm->Data, 0, nackReason);
    }
    rdm->Length = rdm->DataLength + 24; // total packet length

    ++(rdm->CmdClass);
    // Parameter

    uint16_t checkSum = rdmCalculateChecksum(m_rdmBuffer.buffer, rdm->Length);

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
    for (uint16_t i = 0; i < rdm->Length; ++i) {
        m_uart.write(m_rdmBuffer.buffer[i]);
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
                        m_dmxBufferIndex = 0;
                        m_rdmBuffer.buffer[m_dmxBufferIndex] = E120_SC_RDM;
                        ++m_dmxBufferIndex;
                        m_state = State::RDM_RECV;
                        break;
                    default:
                        // ASC
                        m_state = State::IDLE;
                        break;
                }
                break;
            case State::RDM_RECV:
                m_rdmBuffer.buffer[m_dmxBufferIndex] = m_uart.read();

                if (m_dmxBufferIndex >= RDM_BUFFER_SIZE) {
                    if (m_state == State::RDM_RECV) {
                        m_state = State::RDM_RECV_CHECKSUM_HI;
                    } else {
                        m_state = State::IDLE;  // Buffer full
                    }
                } else if (m_dmxBufferIndex >= 2) {
                    // Got enough data to have packet length
                    if ((m_dmxBufferIndex + 1) >= m_rdmBuffer.packet.Length) {
                        // Got expected packet length, need checksum
                        m_state = State::RDM_RECV_CHECKSUM_HI;
                    } else {
                        if (m_rdmBuffer.packet.SubStartCode != E120_SC_SUB_MESSAGE) {
                            m_state = State::IDLE;  // Invalid RDM packet
                        }
                    }
                }

                ++m_dmxBufferIndex;

                break;
            case State::RDM_RECV_CHECKSUM_HI:
                m_rdmChecksum = (m_uart.read() << 8);
                m_state = State::RDM_RECV_CHECKSUM_LO;
                break;
            case State::RDM_RECV_CHECKSUM_LO:
                m_rdmChecksum = (m_rdmChecksum | m_uart.read());
                if (m_rdmChecksum == rdmCalculateChecksum(m_rdmBuffer.buffer, m_rdmBuffer.packet.Length)) {
                    m_state = State::RDM_COMPLETE;
                } else {
                    m_state = State::IDLE;  // Invalid RDM checksum
                }
                break;
            case State::DMX_RECV:
                m_activeBuffer[m_dmxBufferIndex] = m_uart.read();
                ++m_dmxBufferIndex;

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
        if (m_state == RDM_COMPLETE) {
            processRDM();
        }
    }
}
