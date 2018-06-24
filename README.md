The Contiki Operating System
============================

[![Build Status](https://travis-ci.org/contiki-os/contiki.svg?branch=master)](https://travis-ci.org/contiki-os/contiki/branches)

Contiki is an open source operating system that runs on tiny low-power
microcontrollers and makes it possible to develop applications that
make efficient use of the hardware while providing standardized
low-power wireless communication for a range of hardware platforms.

Contiki is used in numerous commercial and non-commercial systems,
such as city sound monitoring, street lights, networked electrical
power meters, industrial monitoring, radiation monitoring,
construction site monitoring, alarm systems, remote house monitoring,
and so on.

For more information, see the Contiki website:

[http://contiki-os.org](http://contiki-os.org)

contiki_sam4s
============================
This is a hobby project. Use with caution :)

The project incorporates support for the sam4s chip from Microchip(Atmel). The radio link is done over an AT86RF231 using the USART’s spi interface. The USB interface enumerates as two ttyACM, where port0 is a SLIP interface and PORT1 can be used for debugging (PRINTF). 
