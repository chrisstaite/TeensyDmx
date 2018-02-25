[![Build Status](https://travis-ci.org/chrisstaite/TeensyDmx.svg?branch=master)](https://travis-ci.org/chrisstaite/TeensyDmx)

TeensyDmx
=========

DMX Transmit and Receive for Teensy 3.x and Teensy-LC with RDM responder
(client) support.  It should support all of the serial interfaces on the
devices.

This is the combination of DMXSerial2, DmxReceive and DmxSimple but
converted to use the UART for Tx rather than the bit-banging method
utilised by DmxSimple.

I've done limited testing on the receive and transmit functionality
with the hardware I have lying around.  DMX transmit, DMX receive
and RDM responder tested working on Serial1, on Teensy 3.2, 3.5 and LC
on Arduino IDE 1.8.2.
