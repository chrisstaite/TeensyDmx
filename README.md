TeensyDmx
=========

DMX Transmit and Receive for Teensy 3.1 with RDM support.

This is the combination of DMXSerial2, DmxReceive and DmxSimple but
converted to use the UART for Tx rather than the bit-banging method
utilised by DmxSimple.

I've done limited testing on the receive and transmit functionality
with the hardware I have lying around.  However I've not got an
RDM controller, so it may or may not work...  It's based on the
working implementation in DMXSerial2, so it *should* work.


An example usage is below:

```
#define DMX_REDE 30
struct RDMINIT rdmData {
  "TeensyDMX v0.1",
  "Teensyduino",
  1,  // Device ID
  "DMX Node",
  1,  // The DMX footprint
  0,  // The DMX startAddress - only used for RDM
  0,  // Additional commands length for RDM
  0   // Definition of additional commands
};
TeensyDmx Dmx(Serial2, &rdmData, DMX_REDE);
```
