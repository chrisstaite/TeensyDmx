#include "TeensyDmx.h"
#include "rdm.h"
#include <limits>

namespace {

constexpr uint32_t BREAKSPEED = 100000;
constexpr uint32_t RDM_BREAKSPEED = 45500;
constexpr uint32_t BREAKFORMAT = SERIAL_8E1;
constexpr uint32_t DMXSPEED = 250000;
constexpr uint32_t DMXFORMAT = SERIAL_8N2;
constexpr uint16_t NACK_WAS_ACK = 0xffff;  // Send an ACK, not a NACK

// The DeviceInfoGetResponse structure (length = 19) has to be responded for
// E120_DEVICE_INFO.  See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DeviceInfoGetResponse
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
} __attribute__((__packed__));  // struct DeviceInfoGetResponse
static_assert((sizeof(DeviceInfoGetResponse) == 19),
              "Invalid size for DeviceInfoGetResponse struct, is it packed?");

// The CommsStatusGetResponse structure (length = 6) has to be responded for
// E120_COMMS_STATUS.  See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=21
struct CommsStatusGetResponse
{
  uint16_t shortMessage;
  uint16_t lengthMismatch;
  uint16_t checksumFail;
} __attribute__((__packed__));  // struct CommsStatusGetResponse
static_assert((sizeof(CommsStatusGetResponse) == 6),
              "Invalid size for CommsStatusGetResponse struct, is it packed?");

struct DiscUniqueBranchRequest
{
  byte lowerBoundUID[RDM_UID_LENGTH];
  byte upperBoundUID[RDM_UID_LENGTH];
} __attribute__((__packed__)); // struct DiscUniqueBranchRequest
static_assert((sizeof(DiscUniqueBranchRequest) == 12),
              "Invalid size for DiscUniqueBranchRequest struct, is it packed?");

// the special discovery response message
struct DiscUniqueBranchResponse
{
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
} __attribute__((__packed__)); // struct DiscUniqueBranchResponse
static_assert((sizeof(DiscUniqueBranchResponse) == 24),
              "Invalid size for DiscUniqueBranchResponse struct, is it packed?");

enum { RDM_PACKET_SIZE_NO_PD = (sizeof(RdmData) - RDM_MAX_PARAMETER_DATA_LENGTH) };
static_assert((RDM_PACKET_SIZE_NO_PD == 24),
              "Invalid size for RDM packet without parameter data");

#if defined(HAS_KINETISK_UART5)
// Instance for UART0, UART1, UART2, UART3, UART4, UART5
TeensyDmx *uartInstances[6] = {0};
#elif defined(HAS_KINETISK_UART4)
// Instance for UART0, UART1, UART2, UART3, UART4
TeensyDmx *uartInstances[5] = {0};
#elif defined(HAS_KINETISK_UART3)
// Instance for UART0, UART1, UART2, UART3
TeensyDmx *uartInstances[4] = {0};
#else
// Instance for UART0, UART1, UART2
TeensyDmx *uartInstances[3] = {0};
#endif

inline uint16_t getUInt16(const byte* const buffer)
{
    return (buffer[0] << 8) | buffer[1];
}

inline uint16_t swapUInt16(uint16_t i)
{
    return (i << 8) | (i >> 8);
}

inline void putUInt16(void* const buffer, const uint16_t value)
{
    reinterpret_cast<byte*>(buffer)[0] = value >> 8;
    reinterpret_cast<byte*>(buffer)[1] = value & 0xff;
}

inline void putUInt32(void* const buffer, const uint32_t value)
{
    reinterpret_cast<byte*>(buffer)[0] = (value & 0xff000000) >> 24;
    reinterpret_cast<byte*>(buffer)[1] = (value & 0x00ff0000) >> 16;
    reinterpret_cast<byte*>(buffer)[2] = (value & 0x0000ff00) >> 8;
    reinterpret_cast<byte*>(buffer)[3] = (value & 0x000000ff);
}

}  // anon namespace

TeensyDmx::TeensyDmx(HardwareSerial& uart, RdmInit* rdm, uint8_t redePin) :
    TeensyDmx(uart, rdm)
{
    pinMode(redePin, OUTPUT);
    m_redePin = portOutputRegister(redePin);
    *m_redePin = 0;
}

TeensyDmx::TeensyDmx(HardwareSerial& uart, RdmInit* rdm) :
    m_uart(uart),
    m_dmxBuffer1{0},
    m_dmxBuffer2{0},
    m_activeBuffer(m_dmxBuffer1),
    m_inactiveBuffer(m_dmxBuffer2),
    m_dmxBufferIndex(0),
    m_frameCount(0),
    m_shortMessage(0),
    m_checksumFail(0),
    m_lengthMismatch(0),
    m_newFrame(false),
    m_rdmChange(false),
    m_mode(DMX_OFF),
    m_state(State::IDLE),
    m_redePin(nullptr),
    m_rdmMute(false),
    m_identifyMode(false),
    m_rdm(rdm),
    m_rdmNeedsProcessing(false),
    m_rdmBuffer(),
    m_rdmChecksum(0),
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

const volatile uint8_t* TeensyDmx::getBuffer() const
{
    if (m_mode == DMX_IN) {
        // DMX Rx is double buffered due to the interrupt handler
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

const volatile uint16_t TeensyDmx::getShortMessage() const
{
    return m_shortMessage;
}

const volatile uint16_t TeensyDmx::getChecksumFail() const
{
    return m_checksumFail;
}

const volatile uint16_t TeensyDmx::getLengthMismatch() const
{
    return m_lengthMismatch;
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
            setDirection(false); // Off puts in receive state so as to be passive
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
        // Send the NSC
        m_uart.begin(DMXSPEED, DMXFORMAT);
        m_uart.write(0);
        m_dmxBufferIndex = 0;
    } else if (m_state == State::DMX_TX) {
        // Check if we're at the end of the packet
        if (m_dmxBufferIndex == DMX_BUFFER_SIZE) {
            m_state = State::BREAK;
            // Send BREAK
            m_uart.begin(BREAKSPEED, BREAKFORMAT);
            m_uart.write(0);
        } else {
            m_uart.write(m_activeBuffer[m_dmxBufferIndex]);
            ++m_dmxBufferIndex;
        }
    }
}

void uart0_status_isr();  // Back reference to serial1.c
void UART0TxStatus()
{
    if ((UART0_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[0]->nextTx();
    }
    // Call standard ISR too
    uart0_status_isr();
}

void uart1_status_isr();  // Back reference to serial2.c
void UART1TxStatus()
{
    if ((UART1_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[1]->nextTx();
    }
    // Call standard ISR too
    uart1_status_isr();
}

void uart2_status_isr();  // Back reference to serial3.c
void UART2TxStatus()
{
    if ((UART2_S1 & UART_S1_TC)) {
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
    if ((UART3_S1 & UART_S1_TC)) {
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
    if ((UART4_S1 & UART_S1_TC)) {
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
    if ((UART5_S1 & UART_S1_TC)) {
        // TX complete
        uartInstances[5]->nextTx();
    }
    // Call standard ISR too
    uart5_status_isr();
}
#endif

void TeensyDmx::startTransmit()
{
    setDirection(true);

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
    bool newFrame = m_newFrame;
    m_newFrame = false;
    return newFrame;
}

bool TeensyDmx::rdmChanged(void)
{
    bool rdmChange = m_rdmChange;
    m_rdmChange = false;
    return rdmChange;
}

void TeensyDmx::completeFrame()
{
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
        case State::RDM_RECV:
        case State::RDM_RECV_CHECKSUM_HI:
            // Double check the previous partial message was an RDM one
            if ((m_mode == DMX_IN) &&
                (m_rdmBuffer.subStartCode == E120_SC_SUB_MESSAGE)) {
                if (m_dmxBufferIndex < 9) {
                    // Destination UID needs 8, but we post increment, hence 9
                    maybeIncrementShortMessage();
                } else if (m_dmxBufferIndex < (m_rdmBuffer.length + 3)) {
                    // Expected length plus checksum, but we post increment, hence 3
                    maybeIncrementLengthMismatch();
                }
            }
            // Fall through
        default:
            // Unknown, ASC? frame or was RDM packet
            break;
    }
    m_state = State::BREAK;
}

void TeensyDmx::rdmDiscUniqueBranch()
{
    if (m_rdm == nullptr || m_rdmMute) {
        return;
    }

    if (m_rdmBuffer.length != (RDM_PACKET_SIZE_NO_PD + sizeof(DiscUniqueBranchRequest))) {
        return;
    }

    if (m_rdmBuffer.dataLength != sizeof(DiscUniqueBranchRequest)) {
        return;
    }

    DiscUniqueBranchRequest *dub_request =
        reinterpret_cast<DiscUniqueBranchRequest*>(m_rdmBuffer.data);

    if (memcmp(dub_request->lowerBoundUID, m_rdm->uid, RDM_UID_LENGTH) <= 0 &&
            memcmp(m_rdm->uid, dub_request->upperBoundUID, RDM_UID_LENGTH) <= 0) {
        // I'm in range - say hello to the lovely controller

        // respond with the special discovery message !
        DiscUniqueBranchResponse *dub_response =
            reinterpret_cast<DiscUniqueBranchResponse*>(&m_rdmBuffer);

        // fill in the discovery response structure
        for (byte i = 0; i < 7; ++i) {
            dub_response->headerFE[i] = 0xFE;
        }
        dub_response->headerAA = 0xAA;
        for (byte i = 0; i < 6; ++i) {
            dub_response->maskedDevID[i+i]   = m_rdm->uid[i] | 0xAA;
            dub_response->maskedDevID[i+i+1] = m_rdm->uid[i] | 0x55;
        }

        uint16_t checksum =
            rdmCalculateChecksum(dub_response->maskedDevID,
                                 sizeof(dub_response->maskedDevID));

        dub_response->checksum[0] = (checksum >> 8)   | 0xAA;
        dub_response->checksum[1] = (checksum >> 8)   | 0x55;
        dub_response->checksum[2] = (checksum & 0xFF) | 0xAA;
        dub_response->checksum[3] = (checksum & 0xFF) | 0x55;

        // Send reply
        stopReceive();
        setDirection(true);
        // No break for DUB
        m_uart.begin(DMXSPEED, DMXFORMAT);
        m_uart.write(reinterpret_cast<uint8_t*>(&m_rdmBuffer), sizeof(DiscUniqueBranchResponse));
        m_uart.flush();
        startReceive();
    }
}

uint16_t TeensyDmx::rdmDiscUnMute()
{
    if (m_rdmBuffer.dataLength != 0) {
        return E120_NR_FORMAT_ERROR;
    }
    m_rdmMute = false;
    // Control field
    m_rdmBuffer.data[0] = 0;
    m_rdmBuffer.data[1] = 0;
    m_rdmBuffer.dataLength = 2;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmDiscMute()
{
    if (m_rdmBuffer.dataLength != 0) {
        return E120_NR_FORMAT_ERROR;
    }
    m_rdmMute = true;
    // Control field
    m_rdmBuffer.data[0] = 0;
    m_rdmBuffer.data[1] = 0;
    m_rdmBuffer.dataLength = 2;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmSetIdentifyDevice()
{
    if (m_rdmBuffer.dataLength != 1) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    }
    if ((m_rdmBuffer.data[0] != 0) && (m_rdmBuffer.data[0] != 1)) {
        // Out of range data
        return E120_NR_DATA_OUT_OF_RANGE;
    }
    m_identifyMode = m_rdmBuffer.data[0] != 0;
    m_rdmChange = true;
    m_rdmBuffer.dataLength = 0;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmSetCommsStatus()
{
    if (m_rdmBuffer.dataLength != 0) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    }
    m_shortMessage = 0;
    m_lengthMismatch = 0;
    m_checksumFail = 0;
    m_rdmChange = true;
    m_rdmBuffer.dataLength = 0;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmSetDeviceLabel()
{
    if (m_rdmBuffer.dataLength > RDM_MAX_STRING_LENGTH) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    }
    memcpy(m_deviceLabel, m_rdmBuffer.data, m_rdmBuffer.dataLength);
    m_deviceLabel[m_rdmBuffer.dataLength] = '\0';
    m_rdmBuffer.dataLength = 0;
    m_rdmChange = true;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmSetDMXStartAddress()
{
    if (m_rdmBuffer.dataLength != 2) {
        // Oversized data
        return E120_NR_FORMAT_ERROR;
    }
    uint16_t newStartAddress = getUInt16(m_rdmBuffer.data);
    if ((newStartAddress <= 0) || (newStartAddress > DMX_BUFFER_SIZE)) {
        // Out of range start address
        return E120_NR_DATA_OUT_OF_RANGE;
    }
    if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    }
    m_rdm->startAddress = newStartAddress;
    m_rdmBuffer.dataLength = 0;
    m_rdmChange = true;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmGetCommsStatus()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    }
    if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    }
    // return all comms status data
    // The data to be responded has to be in the Data buffer.
    CommsStatusGetResponse *commsStatus =
        reinterpret_cast<CommsStatusGetResponse*>(m_rdmBuffer.data);

    putUInt16(&commsStatus->shortMessage, m_shortMessage);
    putUInt16(&commsStatus->lengthMismatch, m_lengthMismatch);
    putUInt16(&commsStatus->checksumFail, m_checksumFail);
    m_rdmBuffer.dataLength = sizeof(CommsStatusGetResponse);
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmGetIdentifyDevice()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    }
    if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    }
    m_rdmBuffer.data[0] = m_identifyMode;
    m_rdmBuffer.dataLength = 1;
    return NACK_WAS_ACK;
}

uint16_t TeensyDmx::rdmGetDeviceInfo()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        // return all device info data
        // The data to be responded has to be in the Data buffer.
        DeviceInfoGetResponse *devInfo =
            reinterpret_cast<DeviceInfoGetResponse*>(m_rdmBuffer.data);

        devInfo->protocolMajor = 1;
        devInfo->protocolMinor = 0;
        devInfo->currentPersonality = 1;
        devInfo->personalityCount = 1;
        devInfo->subDeviceCount = 0;
        devInfo->sensorCount = 0;
        if (m_rdm == nullptr) {
            devInfo->deviceModel = 0;
            putUInt16(&devInfo->productCategory, E120_PRODUCT_CATEGORY_NOT_DECLARED);
            devInfo->softwareVersion = 0;
            devInfo->startAddress = 0;
            devInfo->footprint = 0;
        } else {
            putUInt16(&devInfo->deviceModel, m_rdm->deviceModelId);
            putUInt16(&devInfo->productCategory, m_rdm->productCategory);
            putUInt32(&devInfo->softwareVersion, m_rdm->softwareVersionId);
            putUInt16(&devInfo->startAddress, m_rdm->startAddress);
            putUInt16(&devInfo->footprint, m_rdm->footprint);
        }

        m_rdmBuffer.dataLength = sizeof(DeviceInfoGetResponse);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetManufacturerLabel()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the manufacturer label
        m_rdmBuffer.dataLength = strnlen(m_rdm->manufacturerLabel, RDM_MAX_STRING_LENGTH);
        memcpy(m_rdmBuffer.data, m_rdm->manufacturerLabel, m_rdmBuffer.dataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDeviceModelDescription()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the DEVICE MODEL DESCRIPTION
        m_rdmBuffer.dataLength = strnlen(m_rdm->deviceModel, RDM_MAX_STRING_LENGTH);
        memcpy(m_rdmBuffer.data, m_rdm->deviceModel, m_rdmBuffer.dataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDeviceLabel()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        m_rdmBuffer.dataLength = strnlen(m_deviceLabel, RDM_MAX_STRING_LENGTH);
        memcpy(m_rdmBuffer.data, m_deviceLabel, m_rdmBuffer.dataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetSoftwareVersionLabel()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else if (m_rdm == nullptr) {
        return E120_NR_HARDWARE_FAULT;
    } else {
        // return the SOFTWARE_VERSION_LABEL
        m_rdmBuffer.dataLength = strnlen(m_rdm->softwareLabel, RDM_MAX_STRING_LENGTH);
        memcpy(m_rdmBuffer.data, m_rdm->softwareLabel, m_rdmBuffer.dataLength);
        return NACK_WAS_ACK;
    }
}

uint16_t TeensyDmx::rdmGetDMXStartAddress()
{
   if (m_rdmBuffer.dataLength > 0) {
       // Unexpected data
       return E120_NR_FORMAT_ERROR;
   } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
       // No sub-devices supported
       return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
   } else {
       if (m_rdm == nullptr) {
           putUInt16(m_rdmBuffer.data, 0);
       } else {
           putUInt16(m_rdmBuffer.data, m_rdm->startAddress);
       }
       m_rdmBuffer.dataLength = sizeof(m_rdm->startAddress);
       return NACK_WAS_ACK;
   }
}

uint16_t TeensyDmx::rdmGetSupportedParameters()
{
    if (m_rdmBuffer.dataLength > 0) {
        // Unexpected data
        return E120_NR_FORMAT_ERROR;
    } else if (m_rdmBuffer.subDev != RDM_ROOT_DEVICE) {
        // No sub-devices supported
        return E120_NR_SUB_DEVICE_OUT_OF_RANGE;
    } else {
        m_rdmBuffer.dataLength = 8;
        putUInt16(&m_rdmBuffer.data[0], E120_MANUFACTURER_LABEL);
        putUInt16(&m_rdmBuffer.data[2], E120_DEVICE_MODEL_DESCRIPTION);
        putUInt16(&m_rdmBuffer.data[4], E120_DEVICE_LABEL);
        putUInt16(&m_rdmBuffer.data[6], E120_COMMS_STATUS);
        if (m_rdm != nullptr) {
            for (int n = 0; n < m_rdm->additionalCommandsLength; ++n) {
                putUInt16(&m_rdmBuffer.data[m_rdmBuffer.dataLength],
                          m_rdm->additionalCommands[n]);
                m_rdmBuffer.dataLength += 2;
            }
        }
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

bool TeensyDmx::isForMe(const byte* id)
{
    return (memcmp(id, m_rdm->uid, RDM_UID_LENGTH) == 0);
}

bool TeensyDmx::isForVendor(const byte* id)
{
    if (id[0] != m_rdm->uid[0] || id[1] != m_rdm->uid[1])
    {
        return false;
    }
    for (int i = 2; i < RDM_UID_LENGTH; ++i)
    {
        if (id[i] != 0xff) {
            return false;
        }
    }
    return true;
}

bool TeensyDmx::isForAll(const byte* id)
{
    for (int i = 0; i < RDM_UID_LENGTH; ++i) {
        if (*id != 0xff) {
            return false;
        }
        ++id;
    }
    return true;
}

void TeensyDmx::maybeIncrementShortMessage()
{
    // RDM message and not complete destination UID
    // We only call this when we get a message without a destination UID
    // Ensure we don't overflow
    if (m_shortMessage < std::numeric_limits<uint16_t>::max()) {
        ++m_shortMessage;
    }
}

void TeensyDmx::maybeIncrementLengthMismatch()
{
    // RDM message for me, vendorcast or broadcast where length didn't match message length plus checksum, either too long or too short
    // We only call this when we get a message with an invalid length
    if (isForMe(m_rdmBuffer.destId) || isForAll(m_rdmBuffer.destId) || isForVendor(m_rdmBuffer.destId)) {
        // Ensure we don't overflow
        if (m_lengthMismatch < std::numeric_limits<uint16_t>::max()) {
            ++m_lengthMismatch;
        }
    }
}

void TeensyDmx::maybeIncrementChecksumFail()
{
    // RDM message for me, vendorcast or broadcast where checksum was incorrect
    // We only call this when we get an invalid checksum
    if (isForMe(m_rdmBuffer.destId) || isForAll(m_rdmBuffer.destId) || isForVendor(m_rdmBuffer.destId)) {
          // Ensure we don't overflow
          if (m_checksumFail < std::numeric_limits<uint16_t>::max()) {
              ++m_checksumFail;
          }
    }
}

void TeensyDmx::processRDM()
{
    if (m_rdm == nullptr) {
        return;
    }

    uint16_t nackReason = E120_NR_UNKNOWN_PID;

    m_state = IDLE;
    unsigned long timingStart = micros();

    bool forMe = isForMe(m_rdmBuffer.destId);
    if (forMe || isForAll(m_rdmBuffer.destId) || isForVendor(m_rdmBuffer.destId)) {
        bool sendResponse = true;
        uint16_t parameter = swapUInt16(m_rdmBuffer.parameter);
        if (m_rdmBuffer.cmdClass == E120_DISCOVERY_COMMAND) {
            switch (parameter) {
                case E120_DISC_UNIQUE_BRANCH:
                    rdmDiscUniqueBranch();
                    sendResponse = false;  // DUB is special
                    break;
                case E120_DISC_UN_MUTE:
                    nackReason = rdmDiscUnMute();
                    break;
                case E120_DISC_MUTE:
                    nackReason = rdmDiscMute();
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
        } else if (m_rdmBuffer.cmdClass == E120_GET_COMMAND) {
            switch (parameter) {
                case E120_IDENTIFY_DEVICE:
                    nackReason = rdmGetIdentifyDevice();
                    break;
                case E120_DEVICE_LABEL:
                    nackReason = rdmGetDeviceLabel();
                    break;
                case E120_DMX_START_ADDRESS:
                    nackReason = rdmGetDMXStartAddress();
                    break;
                case E120_SUPPORTED_PARAMETERS:
                    nackReason = rdmGetSupportedParameters();
                    break;
                case E120_DEVICE_INFO:
                    nackReason = rdmGetDeviceInfo();
                    break;
                case E120_MANUFACTURER_LABEL:
                    nackReason = rdmGetManufacturerLabel();
                    break;
                case E120_DEVICE_MODEL_DESCRIPTION:
                    nackReason = rdmGetDeviceModelDescription();
                    break;
                case E120_SOFTWARE_VERSION_LABEL:
                    nackReason = rdmGetSoftwareVersionLabel();
                    break;
                case E120_COMMS_STATUS:
                    nackReason = rdmGetCommsStatus();
                    break;
                default:
                    nackReason = E120_NR_UNKNOWN_PID;
                    break;
            }
        } else if (m_rdmBuffer.cmdClass == E120_SET_COMMAND) {
            switch (parameter) {
                case E120_IDENTIFY_DEVICE:
                    nackReason = rdmSetIdentifyDevice();
                    break;
                case E120_DEVICE_LABEL:
                    nackReason = rdmSetDeviceLabel();
                    break;
                case E120_DMX_START_ADDRESS:
                    nackReason = rdmSetDMXStartAddress();
                    break;
                case E120_COMMS_STATUS:
                    nackReason = rdmSetCommsStatus();
                    break;
                case E120_SUPPORTED_PARAMETERS:
                case E120_DEVICE_INFO:
                case E120_MANUFACTURER_LABEL:
                case E120_DEVICE_MODEL_DESCRIPTION:
                case E120_SOFTWARE_VERSION_LABEL:
                    nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
                    break;
                default:
                    nackReason = E120_NR_UNKNOWN_PID;
                    break;
            }
        } else {
            // Unknown command class
            nackReason = E120_NR_FORMAT_ERROR;
        }
        if (forMe && sendResponse) {
            // TIMING: don't send too fast, min: 176 microseconds
            timingStart = micros() - timingStart;
            if (timingStart < 176) {
                delayMicroseconds(176 - timingStart);
            }
            respondMessage(nackReason);
        }
    }
}

void TeensyDmx::respondMessage(uint16_t nackReason)
{
    // swap SrcID into DestID for sending back.
    memcpy(m_rdmBuffer.destId, m_rdmBuffer.sourceId, RDM_UID_LENGTH);
    if (m_rdm != nullptr) {
        memcpy(m_rdmBuffer.sourceId, m_rdm->uid, RDM_UID_LENGTH);
    } else {
        nackReason = E120_NR_HARDWARE_FAULT;
    }

    // no need to set these data fields:
    // StartCode, SubStartCode
    m_rdmBuffer.messageCount = 0;  // Number of queued messages
    if (nackReason == NACK_WAS_ACK) {
        m_rdmBuffer.responseType = E120_RESPONSE_TYPE_ACK;
    } else {
        m_rdmBuffer.responseType = E120_RESPONSE_TYPE_NACK_REASON;
        m_rdmBuffer.dataLength = 2;
        putUInt16(&m_rdmBuffer.data, nackReason);
    }
    m_rdmBuffer.length = m_rdmBuffer.dataLength + RDM_PACKET_SIZE_NO_PD;  // total packet length

    ++m_rdmBuffer.cmdClass;

    uint16_t checkSum = rdmCalculateChecksum(reinterpret_cast<uint8_t*>(&m_rdmBuffer),
                                             m_rdmBuffer.length);

    // Send reply
    stopReceive();
    setDirection(true);

    m_uart.begin(RDM_BREAKSPEED, BREAKFORMAT);
    m_uart.write(0);
    m_uart.flush();
    m_uart.begin(DMXSPEED, DMXFORMAT);
    m_uart.write(reinterpret_cast<uint8_t*>(&m_rdmBuffer), m_rdmBuffer.length);
    m_uart.flush();
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
    if (UART0_S1 & UART_S1_FE) {
        (void) UART0_D;
    }

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
    if (UART1_S1 & UART_S1_FE) {
        (void) UART1_D;
    }

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
    if (UART2_S1 & UART_S1_FE) {
        (void) UART2_D;
    }

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
    if (UART3_S1 & UART_S1_FE) {
        (void) UART3_D;
    }

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
    if (UART4_S1 & UART_S1_FE) {
        (void) UART4_D;
    }

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
    if (UART5_S1 & UART_S1_FE) {
        (void) UART5_D;
    }

    uartInstances[5]->completeFrame();
}
#endif

void UART0RxStatus()
{
    uint8_t s = UART0_S1;
#ifdef HAS_KINETISK_UART0_FIFO
	if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		uint8_t avail = UART0_RCFIFO;
		if (avail == 0) {
			(void) UART0_D;
			UART0_CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			do {
			    uartInstances[0]->handleByte(UART0_D);
			} while (--avail);
		}
	}
#else
    if (s & UART_S1_FE) {
        (void) UART0_D;
        uartInstances[0]->completeFrame();
    }
    else if (s & UART_S1_RDRF) {
        uartInstances[0]->handleByte(UART0_D);
    }
#endif
    uart0_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART0_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}

void UART1RxStatus()
{
    uint8_t s = UART1_S1;
#ifdef HAS_KINETISK_UART1_FIFO
	if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		uint8_t avail = UART1_RCFIFO;
		if (avail == 0) {
			(void) UART1_D;
			UART1_CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			do {
			    uartInstances[1]->handleByte(UART1_D);
			} while (--avail);
		}
	}
#else
    if (s & UART_S1_FE) {
        (void) UART1_D;
        uartInstances[1]->completeFrame();
    } else if (s & UART_S1_RDRF) {
        uartInstances[1]->handleByte(UART1_D);
    }
#endif
    uart1_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART1_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}

void UART2RxStatus()
{
    uint8_t s = UART2_S1;
#ifdef HAS_KINETISK_UART2_FIFO
	if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		uint8_t avail = UART2_RCFIFO;
		if (avail == 0) {
			(void) UART2_D;
			UART2_CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			do {
			    uartInstances[2]->handleByte(UART2_D);
			} while (--avail);
		}
	}
#else
    if (s & UART_S1_FE) {
        (void) UART2_D;
        uartInstances[2]->completeFrame();
    } else if (s & UART_S1_RDRF) {
        uartInstances[2]->handleByte(UART2_D);
    }
#endif
    uart2_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART2_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}

#ifdef HAS_KINETISK_UART3
void UART3RxStatus()
{
    uint8_t s = UART3_S1;
#ifdef HAS_KINETISK_UART3_FIFO
	if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		uint8_t avail = UART3_RCFIFO;
		if (avail == 0) {
			(void) UART3_D;
			UART3_CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			do {
			    uartInstances[3]->handleByte(UART3_D);
			} while (--avail);
		}
	}
#else
    if (s & UART_S1_FE) {
        (void) UART3_D;
        uartInstances[3]->completeFrame();
    } else if (s & UART_S1_RDRF) {
        uartInstances[3]->handleByte(UART3_D);
    }
#endif
    uart3_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART3_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}
#endif

#ifdef HAS_KINETISK_UART4
void UART4RxStatus()
{
    uint8_t s = UART4_S1;
#ifdef HAS_KINETISK_UART4_FIFO
	if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		uint8_t avail = UART4_RCFIFO;
		if (avail == 0) {
			(void) UART4_D;
			UART4_CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			do {
			    uartInstances[4]->handleByte(UART4_D);
			} while (--avail);
		}
	}
#else
    if (s & UART_S1_FE) {
        (void) UART4_D;
        uartInstances[4]->completeFrame();
    } else if (s & UART_S1_RDRF) {
        uartInstances[4]->handleByte(UART4_D);
    }
#endif
    uart4_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART4_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}
#endif

#ifdef HAS_KINETISK_UART5
void UART5RxStatus()
{
    uint8_t s = UART5_S1;
#ifdef HAS_KINETISK_UART5_FIFO
    if (s & (UART_S1_RDRF | UART_S1_IDLE)) {
        __disable_irq();
        uint8_t avail = UART5_RCFIFO;
        if (avail == 0) {
            (void) UART5_D;
            UART5_CFIFO = UART_CFIFO_RXFLUSH;
            __enable_irq();
        } else {
            __enable_irq();
            do {
                uartInstances[5]->handleByte(UART5_D);
            } while (--avail);
        }
    }
#else
    if (s & UART_S1_FE) {
        (void) UART5_D;
        uartInstances[5]->completeFrame();
    } else if (s & UART_S1_RDRF) {
        uartInstances[5]->handleByte(UART5_D);
    }
#endif
    uart5_status_isr();
    // Reset all flags on Teensy-LC
#ifdef KINETISL
    UART5_S1 = UART_S1_IDLE | UART_S1_OR | UART_S1_NF | UART_S1_FE | UART_S1_PF;
#endif
}
#endif

void TeensyDmx::startReceive()
{
    setDirection(false);

    // UART Initialisation
    m_uart.begin(250000);

    if (&m_uart == &Serial1) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART0 receive interrupt immediately after each byte received
#ifdef HAS_KINETISK_UART0_FIFO
        UART0_RWFIFO = 1;
#endif

        // Enable UART0 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART0_FIFO
        UART0_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART0_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);

        attachInterruptVector(IRQ_UART0_STATUS, UART0RxStatus);

#ifdef HAS_KINETISK_UART0_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART0_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART0_STATUS) + 1);

        // Enable UART0 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
        attachInterruptVector(IRQ_UART0_ERROR, UART0RxError);
#endif
    } else if (&m_uart == &Serial2) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART1 receive interrupt immediately after each byte received
#ifdef HAS_KINETISK_UART1_FIFO
        UART1_RWFIFO = 1;
#endif

        // Enable UART0 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART1_FIFO
        UART1_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART1_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);

        attachInterruptVector(IRQ_UART1_STATUS, UART1RxStatus);

#ifdef HAS_KINETISK_UART1_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART1_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART1_STATUS) + 1);

        // Enable UART0 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
        attachInterruptVector(IRQ_UART1_ERROR, UART1RxError);
#endif
    } else if (&m_uart == &Serial3) {
        // Change interrupt vector to mine to monitor RX complete
        // Fire UART2 receive interrupt immediately after each byte received
#ifdef HAS_KINETISK_UART2_FIFO
        UART2_RWFIFO = 1;
#endif

        // Enable UART2 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART2_FIFO
        UART2_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART2_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);

        attachInterruptVector(IRQ_UART2_STATUS, UART2RxStatus);

#ifdef HAS_KINETISK_UART2_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART2_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART2_STATUS) + 1);

        // Enable UART0 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
        attachInterruptVector(IRQ_UART2_ERROR, UART2RxError);
#endif
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        // Fire UART3 receive interrupt immediately after each byte received
        UART3_RWFIFO = 1;

        // Enable UART3 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART3_FIFO
        UART3_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART3_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);

        attachInterruptVector(IRQ_UART3_STATUS, UART3RxStatus);

#ifdef HAS_KINETISK_UART3_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART3_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART3_STATUS) + 1);

        // Enable UART3 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);

        attachInterruptVector(IRQ_UART3_ERROR, UART3RxError);
#endif
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        // Fire UART4 receive interrupt immediately after each byte received
        UART4_RWFIFO = 1;

        // Enable UART4 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART4_FIFO
        UART4_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART4_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART4_STATUS);

        attachInterruptVector(IRQ_UART4_STATUS, UART4RxStatus);

#ifdef HAS_KINETISK_UART4_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART4_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART4_STATUS) + 1);

        // Enable UART2 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);

        attachInterruptVector(IRQ_UART4_ERROR, UART4RxError);
#endif
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        // Fire UART5 receive interrupt immediately after each byte received
        UART5_RWFIFO = 1;

        // Enable UART5 interrupt on frame error and enable IRQ
#ifdef HAS_KINETISK_UART5_FIFO
        UART5_C3 |= UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE;
#else
        UART5_C3 |= UART_C3_FEIE | UART_C2_RIE;
#endif
        NVIC_ENABLE_IRQ(IRQ_UART5_STATUS);

        attachInterruptVector(IRQ_UART5_STATUS, UART5RxStatus);

#ifdef HAS_KINETISK_UART5_FIFO
        // Set error IRQ priority lower than that of the status IRQ,
        // so that the status IRQ receives any leftover bytes before
        // we detect and trigger a new frame.
        NVIC_SET_PRIORITY(IRQ_UART5_ERROR,
                          NVIC_GET_PRIORITY(IRQ_UART5_STATUS) + 1);

        // Enable UART5 interrupt on frame error and enable IRQ
        NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);

        attachInterruptVector(IRQ_UART5_ERROR, UART5RxError);
#endif
    }
#endif

    m_state = State::IDLE;
}

void TeensyDmx::stopReceive()
{
    m_uart.end();

    if (&m_uart == &Serial1) {
#ifdef HAS_KINETISK_UART0_FIFO
        UART0_RWFIFO = 0;
#endif
#ifdef HAS_KINETISK_UART0_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
        attachInterruptVector(IRQ_UART0_ERROR, uart0_error_isr);
        UART0_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART0_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART0_STATUS, uart0_status_isr);
    } else if (&m_uart == &Serial2) {
#ifdef HAS_KINETISK_UART1_FIFO
        UART1_RWFIFO = 0;
#endif
#ifdef HAS_KINETISK_UART1_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
        attachInterruptVector(IRQ_UART1_ERROR, uart1_error_isr);
        UART1_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART1_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART1_STATUS, uart1_status_isr);
    } else if (&m_uart == &Serial3) {
#ifdef HAS_KINETISK_UART2_FIFO
        UART2_RWFIFO = 0;
#endif
#ifdef HAS_KINETISK_UART2_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
        attachInterruptVector(IRQ_UART2_ERROR, uart2_error_isr);
        UART2_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART2_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART2_STATUS, uart2_status_isr);
    }
#ifdef HAS_KINETISK_UART3
    else if (&m_uart == &Serial4) {
        UART3_RWFIFO = 0;
#ifdef HAS_KINETISK_UART3_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
        attachInterruptVector(IRQ_UART3_ERROR, uart3_error_isr);
        UART3_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART3_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART3_STATUS, uart3_status_isr);
    }
#endif
#ifdef HAS_KINETISK_UART4
    else if (&m_uart == &Serial5) {
        UART4_RWFIFO = 0;
#ifdef HAS_KINETISK_UART4_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
        attachInterruptVector(IRQ_UART4_ERROR, uart4_error_isr);
        UART4_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART4_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART4_STATUS, uart4_status_isr);
    }
#endif
#ifdef HAS_KINETISK_UART5
    else if (&m_uart == &Serial6) {
        UART5_RWFIFO = 0;
#ifdef HAS_KINETISK_UART5_FIFO
        NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
        attachInterruptVector(IRQ_UART5_ERROR, uart5_error_isr);
        UART5_C3 &= ~(UART_C3_FEIE | UART_C2_RIE | UART_C2_ILIE);
#else
        UART5_C3 &= ~(UART_C3_FEIE | UART_C2_RIE);
#endif
        attachInterruptVector(IRQ_UART5_STATUS, uart5_status_isr);
    }
#endif
}

inline void TeensyDmx::setDirection(bool transmit) {
    if (m_redePin != nullptr) {
        *m_redePin = (transmit ? 1 : 0);
    }
}

void TeensyDmx::handleByte(uint8_t c)
{
    switch (m_state)
    {
        case State::BREAK:
            switch (c)
            {
                case 0:
                    m_state = State::DMX_RECV;
                    // In DMX mode we don't keep the start code
                    m_dmxBufferIndex = 0;
                    break;
                case E120_SC_RDM:
                    m_rdmNeedsProcessing = false;
                    // Store the start code and then increment
                    reinterpret_cast<uint8_t*>(&m_rdmBuffer)[0] = c;
                    m_dmxBufferIndex = 1;
                    m_state = State::RDM_RECV;
                    break;
                default:
                    // ASC
                    m_state = State::IDLE;
                    break;
            }
            break;
        case State::RDM_RECV:
            reinterpret_cast<uint8_t*>(&m_rdmBuffer)[m_dmxBufferIndex] = c;
            ++m_dmxBufferIndex;
            if (m_dmxBufferIndex >= sizeof(RdmData)) {
                m_state = State::RDM_RECV_CHECKSUM_HI;
            } else if (m_dmxBufferIndex >= 3) {
                // Got enough data to have packet length
                if (m_rdmBuffer.subStartCode != E120_SC_SUB_MESSAGE) {
                    m_state = State::IDLE;  // Invalid RDM packet
                } else if (m_dmxBufferIndex >= m_rdmBuffer.length) {
                    // Got expected packet length, need checksum
                    m_state = State::RDM_RECV_CHECKSUM_HI;
                }
            }
            break;
        case State::RDM_RECV_CHECKSUM_HI:
            m_rdmChecksum = (c << 8);
            ++m_dmxBufferIndex;
            m_state = State::RDM_RECV_CHECKSUM_LO;
            break;
        case State::RDM_RECV_CHECKSUM_LO:
            m_rdmChecksum = (m_rdmChecksum | c);
            ++m_dmxBufferIndex;
            if (m_rdmChecksum ==
                    rdmCalculateChecksum(reinterpret_cast<uint8_t*>(&m_rdmBuffer),
                                         m_rdmBuffer.length)) {
                m_rdmNeedsProcessing = true;
            } else {
                maybeIncrementChecksumFail();
            }
            m_state = State::RDM_RECV_POST_CHECKSUM;
            break;
        case State::RDM_RECV_POST_CHECKSUM:
            maybeIncrementLengthMismatch();
            m_state = State::IDLE;
            break;
        case State::DMX_RECV:
            m_activeBuffer[m_dmxBufferIndex] = c;
            ++m_dmxBufferIndex;
            if (m_dmxBufferIndex >= DMX_BUFFER_SIZE) {
                m_state = State::DMX_COMPLETE;
            }
            break;
        default:
            // Discarding bytes
            break;
    }
}

void TeensyDmx::loop()
{
    if (m_rdmNeedsProcessing)
    {
        m_rdmNeedsProcessing = false;
        processRDM();
    }
}
