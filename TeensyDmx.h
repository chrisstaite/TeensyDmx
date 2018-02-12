/* TeensyDmx - DMX Sender/Receiver with RDM for Teensy 3.2
   Copyright (c) 2017-2018 Peter Newman, Dan Large, http://daniellarge.co.uk
   Copyright (c) 2017 Chris Staite
   Copyright (c) 2014 Jim Paris
   Copyright (c) 2014 Ward
   Copyright (c) 2008-2009 Peter Knight, Tinker.it! All rights reserved.
   Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

#ifndef _TEENSYDMX_H
#define _TEENSYDMX_H

#include "Arduino.h"

enum { DMX_BUFFER_SIZE = 512 };
enum { RDM_UID_LENGTH = 6 };
enum { RDM_MAX_STRING_LENGTH = 32 };
enum { RDM_MAX_PARAMETER_DATA_LENGTH = 231 };
enum { RDM_ROOT_DEVICE = 0 };

enum CallbackStatus { CB_SUCCESS, CB_RDM_BROADCAST, CB_RDM_TIMEOUT, CB_RDM_CHECKSUM_ERROR };

struct RdmData
{
  byte     startCode;    // Start Code 0xCC for RDM
  byte     subStartCode; // Sub Start Code 0x01 for RDM
  byte     length;       // packet length
  byte     destId[RDM_UID_LENGTH];
  byte     sourceId[RDM_UID_LENGTH];

  byte     transNo;     // transaction number, not checked
  byte     responseType;    // ResponseType or PortID
  byte     messageCount;     // number of queued messages
  uint16_t subDev;      // sub device number (root = 0)
  byte     cmdClass;     // command class
  uint16_t parameter;	   // parameter ID
  byte     dataLength;   // parameter data length in bytes
  byte     data[RDM_MAX_PARAMETER_DATA_LENGTH];   // data byte field
} __attribute__((__packed__)); // struct RdmData
static_assert((sizeof(RdmData) == 255),
              "Invalid size for RdmData struct, is it packed?");

typedef void (*rdmControllerCallback)(CallbackStatus, RdmData*);

typedef void (*rdmDiscoveryCallback)(CallbackStatus, byte*, uint32_t);

struct RdmInit
{
    const byte *uid;
    const uint32_t softwareVersionId;
    const char *softwareLabel;
    const char *manufacturerLabel;
    const uint16_t deviceModelId;
    const char *deviceModel;
    const uint16_t productCategory;
    uint16_t footprint;
    uint16_t startAddress;
    const uint16_t additionalCommandsLength;
    const uint16_t *additionalCommands;
    rdmDiscoveryCallback discoveryCallback;
    rdmControllerCallback controllerCallback;
};

class TeensyDmx
{
  public:
    enum Mode { DMX_OFF, DMX_IN, DMX_OUT };

    TeensyDmx(HardwareSerial& uart, struct RdmInit* rdm, uint8_t redePin);

    TeensyDmx(HardwareSerial& uart, struct RdmInit* rdm);

    TeensyDmx(HardwareSerial& uart, uint8_t redePin) :
        TeensyDmx(uart, nullptr, redePin)
    { }

    TeensyDmx(HardwareSerial& uart) :
        TeensyDmx(uart, nullptr)
    { }

    void setMode(TeensyDmx::Mode mode);
    void loop();

    // Returns true if a new frame has been received since the this was last called
    bool newFrame();
    // Get the buffer with the current channel data in
    const volatile uint8_t* getBuffer() const;
    // Use for receive with addresses from 0-511
    uint8_t getChannel(const uint16_t address);
    // Use for receive with addresses from 1-512
    uint8_t getDmxChannel(const uint16_t address)
    {
        return getChannel(address - 1);
    }
    // Returns true if RDM has changed since this was last called
    bool rdmChanged();
    // Returns true if the device should be in identify mode
    bool isIdentify() const;
    // Returns the user-set label of the device
    // This will always be null terminated, therefore can be up to 33 chars
    const char* getLabel() const;

    // Return the comms status error counters
    const volatile uint16_t getShortMessage() const;
    const volatile uint16_t getChecksumFail() const;
    const volatile uint16_t getLengthMismatch() const;

    // Use for transmit with addresses from 0-511
    // Will keep all other values as they were previously
    void setChannel(const uint16_t address, const uint8_t value);
    // Use for transmit with addresses from 1-512
    // Will keep all other values as they were previously
    void setDmxChannel(const uint16_t address, const uint8_t value)
    {
        setChannel(address - 1, value);
    }

    // Use for transmit with channels from 0-511
    // Will set all other channels to 0
    void setChannels(const uint16_t startAddress, const uint8_t* values, const uint16_t length);
    // Use for transmit with channels from 1-512
    // Will set all other channels to 0
    void setDmxChannels(const uint16_t startAddress, const uint8_t* values, const uint16_t length)
    {
        setChannels(startAddress - 1, values, length);
    }

    void sendRDMDiscMute(byte *uid);
    void sendRDMDiscUnMute(byte *uid);
    void sendRDMDiscUniqueBranch(byte *lower_uid, byte *upper_uid);
    void sendRDMGetDeviceInfo(byte *uid);
    void sendRDMSetIdentifyDevice(byte *uid, bool identify_device);
    void sendRDMGetDmxStartAddress(byte *uid);
    void sendRDMSetDmxStartAddress(byte *uid, uint16_t dmx_address);
    void sendRDMGetDmxPersonality(byte *uid);
    void sendRDMSetDmxPersonality(byte *uid, uint8_t personality);
    void sendRDMGetDmxPersonalityDescription(byte *uid, uint8_t personality);
    void sendRDMGetManufacturerLabel(byte *uid);
    void sendRDMGetDeviceLabel(byte *uid);
    void sendRDMGetDeviceModelDescription(byte *uid);

  private:
    TeensyDmx(const TeensyDmx&);
    TeensyDmx& operator=(const TeensyDmx&);

    enum State { IDLE, BREAK,
                 DMX_TX,
                 DMX_RECV, DMX_COMPLETE,
                 RDM_RECV, RDM_RECV_CHECKSUM_HI, RDM_RECV_CHECKSUM_LO, RDM_RECV_POST_CHECKSUM,
                 RDM_DUB_PREAMBLE, RDM_DUB_RECV, RDM_DUB_CHECKSUM_3, RDM_DUB_CHECKSUM_2, RDM_DUB_CHECKSUM_1, RDM_DUB_CHECKSUM_0, RDM_DUB_POST_CHECKSUM };

    enum ControllerState { CONTROLLER_IDLE, RDM_DUB, RDM_DUB_COLLISION, RDM_BROADCAST, RDM_TIMEOUT, RDM_CHECKSUM_ERROR, RDM_MESSAGE };

    void startTransmit();
    void stopTransmit();
    void startReceive();
    void stopReceive();

    void setDirection(bool transmit);

    void completeFrame();  // Called at error ISR during recv
    void processRDM();
    void processDiscovery();
    void respondMessage(uint16_t nackReason);
    void buildSendRDMMessage(byte *uid, uint8_t commandClass, uint16_t pid);
    void sendRDMMessage();
    void handleByte(uint8_t c);

    void nextTx();

    // RDM handler functions
    void rdmDiscUniqueBranch();
    uint16_t rdmDiscMute();
    uint16_t rdmDiscUnMute();
    uint16_t rdmGetCommsStatus();
    uint16_t rdmSetCommsStatus();
    uint16_t rdmGetDeviceInfo();
    uint16_t rdmGetDeviceLabel();
    uint16_t rdmSetDeviceLabel();
    uint16_t rdmGetDeviceModelDescription();
    uint16_t rdmGetDMXStartAddress();
    uint16_t rdmSetDMXStartAddress();
    uint16_t rdmGetIdentifyDevice();
    uint16_t rdmSetIdentifyDevice();
    uint16_t rdmGetManufacturerLabel();
    uint16_t rdmGetSoftwareVersionLabel();
    uint16_t rdmGetSupportedParameters();

    uint16_t rdmCalculateChecksum(uint8_t* data, uint8_t length);
    bool isForMe(const byte* id);
    bool isForVendor(const byte* id);
    bool isForAll(const byte* id);

    void maybeIncrementShortMessage();
    void maybeIncrementChecksumFail();
    void maybeIncrementLengthMismatch();

    HardwareSerial& m_uart;

    volatile uint8_t m_dmxBuffer1[DMX_BUFFER_SIZE];
    volatile uint8_t m_dmxBuffer2[DMX_BUFFER_SIZE];
    volatile uint8_t *m_activeBuffer;
    volatile uint8_t *m_inactiveBuffer;
    volatile uint16_t m_dmxBufferIndex;
    volatile unsigned int m_frameCount;
    volatile uint16_t m_shortMessage;
    volatile uint16_t m_checksumFail;
    volatile uint16_t m_lengthMismatch;
    volatile bool m_newFrame;
    volatile bool m_rdmChange;
    Mode m_mode;
    State m_state;
    ControllerState m_controllerState;
    volatile uint8_t* m_redePin;
    bool m_rdmMute;
    bool m_identifyMode;
    RdmInit *m_rdm;
    volatile bool m_rdmNeedsProcessing;
    RdmData m_rdmBuffer;
    uint16_t m_rdmChecksum;
    // Allow an extra byte for a null if we have a 32 character string
    char m_deviceLabel[RDM_MAX_STRING_LENGTH + 1];
    static_assert((sizeof(m_deviceLabel) == 33), "Invalid size for m_deviceLabel");

    friend void UART0RxStatus(void);
    friend void UART0TxStatus(void);
    friend void UART1RxStatus(void);
    friend void UART1TxStatus(void);
    friend void UART2RxStatus(void);
    friend void UART2TxStatus(void);
#ifndef IRQ_UART0_ERROR
    friend void UART0RxError(void);
    friend void UART1RxError(void);
    friend void UART2RxError(void);
#endif
#ifdef HAS_KINETISK_UART3
    friend void UART3RxStatus(void);
    friend void UART3TxStatus(void);
    friend void UART3RxError(void);
#endif
#ifdef HAS_KINETISK_UART4
    friend void UART4RxStatus(void);
    friend void UART4TxStatus(void);
    friend void UART4RxError(void);
#endif
#ifdef HAS_KINETISK_UART5
    friend void UART5RxStatus(void);
    friend void UART5TxStatus(void);
    friend void UART5RxError(void);
#endif
};

#endif  // _TEENSYDMX_H
