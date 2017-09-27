/* TeensyDmx - DMX Sender/Receiver with RDM for Teensy 3.2
   Copyright (c) 2017 Peter Newman, Dan Large, http://daniellarge.co.uk
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

#ifndef UART_C3_FEIE
#define UART_C3_FEIE    (uint8_t)0x02   // Framing Error Interrupt Enable
#endif

// ----- macros -----

// 16-bit and 32-bit integers in the RDM protocol are transmitted highbyte - lowbyte.
// but the ATMEGA processors store them in highbyte - lowbyte order.
// Use SWAPINT to swap the 2 bytes of an 16-bit int to match the byte order on the DMX Protocol.
// avoid using this macro on variables but use it on the constant definitions.
#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))
// Use SWAPINT32 to swap the 4 bytes of a 32-bit int to match the byte order on the DMX Protocol.
#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)

// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))

enum { DMX_BUFFER_SIZE = 512 };
enum { RDM_UID_LENGTH = 6 };
enum { RDM_MAX_PARAMETER_DATA_LENGTH = 231 };

struct RDMDATA
{
  byte     StartCode;    // Start Code 0xCC for RDM
  byte     SubStartCode; // Sub Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[RDM_UID_LENGTH];
  byte     SourceID[RDM_UID_LENGTH];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType or PortID
  byte     MessageCount;     // number of queued messages
  uint16_t SubDev;      // sub device number (root = 0)
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[RDM_MAX_PARAMETER_DATA_LENGTH];   // data byte field
} __attribute__((__packed__)); // struct RDMDATA
static_assert((sizeof(RDMDATA)==255), "Invalid size for RDMDATA struct, is it packed?");

enum { RDM_BUFFER_SIZE = sizeof(RDMDATA) };  // base packet + param data (no checksum)
enum { RDM_PACKET_SIZE_NO_PD = (sizeof(RDMDATA) - RDM_MAX_PARAMETER_DATA_LENGTH) };
static_assert((RDM_PACKET_SIZE_NO_PD==24), "Invalid size for RDM packet without parameter data");

struct DISC_UNIQUE_BRANCH_REQUEST
{
  byte lowerBoundUID[RDM_UID_LENGTH];
  byte upperBoundUID[RDM_UID_LENGTH];
} __attribute__((__packed__)); // struct DISC_UNIQUE_BRANCH_REQUEST
static_assert((sizeof(DISC_UNIQUE_BRANCH_REQUEST)==12), "Invalid size for DISC_UNIQUE_BRANCH_REQUEST struct, is it packed?");

// the special discovery response message
struct DISC_UNIQUE_BRANCH_RESPONSE
{
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
} __attribute__((__packed__)); // struct DISC_UNIQUE_BRANCH_RESPONSE
static_assert((sizeof(DISC_UNIQUE_BRANCH_RESPONSE)==24), "Invalid size for DISC_UNIQUE_BRANCH_RESPONSE struct, is it packed?");

// The buffer for RDM packets being received and sent.
// this structure is needed to seperate RDM data from DMX data.
union RDMMSG {
  // the most common RDM packet layout for commands
  struct RDMDATA packet;

  // the layout of the RDM packet when sending a DUB response
  struct DISC_UNIQUE_BRANCH_RESPONSE discovery;

  // the byte array used while receiving and sending.
  byte buffer[RDM_BUFFER_SIZE];
} __attribute__((__packed__)); // union RDMMEM

struct RDMINIT
{
    const char *softwareLabel;
    const char *manufacturerLabel;
    const uint16_t deviceModelId;
    const char  *deviceModel;
    uint16_t footprint;
    uint16_t startAddress;
    const uint16_t  additionalCommandsLength;
    const uint16_t  *additionalCommands;
}; // struct RDMINIT

class TeensyDmx
{
  public:
    enum Mode { DMX_OFF, DMX_IN, DMX_OUT };
    TeensyDmx(HardwareSerial& uart);
    TeensyDmx(HardwareSerial& uart, uint8_t redePin);
    TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm);
    TeensyDmx(HardwareSerial& uart, struct RDMINIT* rdm, uint8_t redePin);
    void setMode(TeensyDmx::Mode mode);
    void loop();

    // Returns true if a new frame has been received since the this was last called
    bool newFrame();
    // Use for receive
    const volatile uint8_t* getBuffer() const;
    // Use for receive with addresses from 0-511
    uint8_t getChannel(const uint16_t address);
    // Use for receive with addresses from 1-512
    uint8_t getDmxChannel(const uint16_t address)
    {
        return getChannel(address - 1, value);
    }
    // Returns true if RDM has changed since this was last called
    bool rdmChanged();
    // Returns true if the device should be in identify mode
    bool isIdentify() const;
    // Returns the user-set label of the device
    const char* getLabel() const;

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

  private:
    TeensyDmx(const TeensyDmx&);
    TeensyDmx& operator=(const TeensyDmx&);

    enum State { IDLE, BREAK, DMX_TX, DMX_RECV, DMX_COMPLETE, RDM_RECV, RDM_RECV_CHECKSUM_HI, RDM_RECV_CHECKSUM_LO, RDM_COMPLETE };

    void startTransmit();
    void stopTransmit();
    void startReceive();
    void stopReceive();

    void completeFrame();  // Called at error ISR during recv
    void processRDM();
    void respondMessage(unsigned long timingStart, uint16_t nackReason);
    void readBytes();  // Recv handler

    void nextTx();

    // RDM handler functions
    void rdmDiscUniqueBranch(struct RDMDATA* rdm);
    uint16_t rdmDiscUnMute(struct RDMDATA* rdm);
    uint16_t rdmDiscMute(struct RDMDATA* rdm);
    uint16_t rdmSetIdentifyDevice(struct RDMDATA* rdm);
    uint16_t rdmSetDeviceLabel(struct RDMDATA* rdm);
    uint16_t rdmSetDMXStartAddress(struct RDMDATA* rdm);
    uint16_t rdmGetIdentifyDevice(struct RDMDATA* rdm);
    uint16_t rdmGetDeviceInfo(struct RDMDATA* rdm);
    uint16_t rdmGetManufacturerLabel(struct RDMDATA* rdm);
    uint16_t rdmGetDeviceModelDescription(struct RDMDATA* rdm);
    uint16_t rdmGetDeviceLabel(struct RDMDATA* rdm);
    uint16_t rdmGetSoftwareVersionLabel(struct RDMDATA* rdm);
    uint16_t rdmGetDMXStartAddress(struct RDMDATA* rdm);
    uint16_t rdmGetSupportedParameters(struct RDMDATA* rdm);

    uint16_t rdmCalculateChecksum(uint8_t* data, uint8_t length);

    HardwareSerial& m_uart;

    volatile uint8_t m_dmxBuffer1[DMX_BUFFER_SIZE];
    volatile uint8_t m_dmxBuffer2[DMX_BUFFER_SIZE];
    volatile uint8_t *m_activeBuffer;
    volatile uint8_t *m_inactiveBuffer;
    volatile uint16_t m_dmxBufferIndex;
    volatile unsigned int m_frameCount;
    volatile bool m_newFrame;
    volatile bool m_rdmChange;
    Mode m_mode;
    State m_state;
    volatile uint8_t* m_redePin;
    bool m_rdmMute;
    bool m_identifyMode;
    struct RDMINIT *m_rdm;
    union RDMMSG m_rdmBuffer;
    uint16_t m_rdmChecksum;
    char m_deviceLabel[32];

#ifndef IRQ_UART0_ERROR
    friend void UART0RxStatus(void);
    friend void UART1RxStatus(void);
    friend void UART2RxStatus(void);
#endif
    friend void UART0TxStatus(void);
    friend void UART1TxStatus(void);
    friend void UART2TxStatus(void);
    friend void UART0RxError(void);
    friend void UART1RxError(void);
    friend void UART2RxError(void);
#ifdef HAS_KINETISK_UART3
    friend void UART3TxStatus(void);
    friend void UART3RxError(void);
#endif
#ifdef HAS_KINETISK_UART4
    friend void UART4TxStatus(void);
    friend void UART4RxError(void);
#endif
#ifdef HAS_KINETISK_UART5
    friend void UART5TxStatus(void);
    friend void UART5RxError(void);
#endif
};

#endif  // _TEENSYDMX_H
