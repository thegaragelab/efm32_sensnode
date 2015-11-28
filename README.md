# EFM32 SensNode Prototype

This repository consists of the code and hardware design for my entry into the
[Silicon Labs Low Power Contest 2015](http://community.silabs.com/lowpowercontest).

## Overview

This entry is a component of a larger project called [Sensaura](http://sensaura.org)
which provides hardware and software solutions to implement low power sensors
communicating over a wireless network protocol and publish values to a [MQTT]()
message bus.

This project consists of a prototype implementation of a [SensNode](http://sensnode.sensauara.org)
processor module using a [EFM32 Zero Gecko](https://www.silabs.com/products/mcu/lowpower/Pages/efm32zg-stk3200.aspx)
starter kit and a simple sensor backpack to test the power consumption and
functionality of the design.

## Usefulness

The goal is to develop a small, cheap, wireless sensor board with a well defined
[API](http://sensnode.sensaura.org/apidocs/sensnode), [footprint](http://sensnode.sensaura.org/pages/hardware.html)
and [network protocol](http://sensnet.sensaura.org) that can be used as the basis
for a range of customised sensors.

The sensors are intended to be run from batteries (the ideal design should
operate for 6 months on a single AA cell) or other disconnected power sources
which allows for deployment in areas without a fixed power supply available.

Sensor data is gathered through an adapter connected to an IP connected host,
formatted as JSON and published to an MQTT server for consumption. This design
allows maximum flexibility without requiring the sensors themselves to be
connected to the internet.

## Low Power Features

To achieve the desired goal of operating on a single AA cell for a period of
6 months power usage must be carefully carefully controlled both in the design
of the sensor and peripheral circuitry as well as by utilising the low power
features of the EFM32 CPU.

The processor module is anticipated to spend the majority of it's time in
sleep mode, only waking up at periodic intervals to read sensors and transmit
results. In the case of the EFM32 this will set the processor to energy mode 3
(EM3) which allows it to wake up on RTC or pin change events at the discretion
of the firmware.

As the design is generic and intended for use with other ARM processors as well
not all low power features of the EFM32 (such as LeSENSE) can be utilitised due
to difficulty exposing those features in a generic way.

## Implementation

Due to time constraints the implementation of the prototype is not yet complete,
some functionality is untested or only partially implemented. The current code
base is [available on GitHub]() and will be updated as more progress is made.

### Hardware

The hardware components consist of the [Silicon Labs EFM32ZG-STK3200 Development Kit](https://www.silabs.com/products/mcu/lowpower/Pages/efm32zg-stk3200.aspx),
a breakout board matching the final board footprint with a NRF24L01+ wireless module
fitted and jumper leads to connect to the IO pins on the development board and
a simple sensor backpack for testing.

The sensor backpack consists of the following components:

  * [Si7020 Temperature/Humidity Sensor](http://www.silabs.com/products/sensors/humidity-sensors/Pages/si7013-20-21.aspx)
  * [TSL2561T Light Sensor](http://ams.com/eng/Products/Light-Sensors/Ambient-Light-Sensor-ALS/TSL2561)
  * A soil moisture sensor made of two metallic probes configured for resistive measurement.

Two of the components use an I2C interface for communication, the moisture sensor
requires a digital output to control the current flow for measurement and an
analog input to take the reading.

Both boards are single sided PCBs that were milled with a small CNC machine. One
of the goals of the project is to ensure that processor and sensor boards can be
made using resources generally available to the hobbiest - milling or etching
single side boards that can be hand soldered is a core requirement.

To allow data being transmitted on the NRF24L01 network to be captured and used
on a PC a simple [Arduino](https://www.arduino.cc/) based adapter was also constructed.
This simply uses an Arduino Nano to report any packets received on the NRF24L01
network over a serial port connection and was assembled on a breadboard.

### Software

As the [SensNode](http://sensnode.sensaura.org) is intended to be processor
agnostic I use a [simple API](http://sensnode.sensaura.org/apidocs/sensnode) that
provides generic access to GPIO, analog and timing functions. For the purposes
of the prototype I implemented a shim layer that implements the SensNode API
using the [EMLIB Library](http://devtools.silabs.com/dl/documentation/doxygen/EM_CMSIS_P1_DOC_4.0.0/emlib_zero/html/index.html).
The actual sensor firmware was then implemented with calls to the SensNode API.

Implementing a sensor like this is relatively simple in concept - the main
application loop simply reads the values of the various sensors and transmits
them over the NRF24L01 network and then puts the processor to sleep for a specified
period of time before repeating the process.

For this particular sensor there are no unpredictable events (such as motion
sensors) so the wake up process is purely driven by the real time clock. The
values measured in this case are also fairly slow to change (there would not be
a significant change over a duration measured in minutes) so it can spend the
majority of it's time in sleep mode.

## Conclusion

As I mentioned earlier the project is not yet complete. Time constraints and
some unexpected difficulty with the NRF24L01 interface prohibited a full
implementation from being completed in time for this submission. As a result
any power profile readings taken at this stage would not be of much use.

On the positive side however the project has exposed some improvements that can
be made to the overall design;

1. The main issue I had with the NRF24L01 module was the stability of the signals
   on the SPI bus. Most of these are caused by the use of jumper wires to connect
   it to the development board - a common problem when working with external
   peripherals.
2. Due to a misreading of the datasheet for the CPU I assumed that the LEUART
   could only be driven by the 32.768kHz clock and only supported 9600 baud
   transmission. As a result I used the two USART channels to implement I2C and
   serial communications and provided a bit banged SPI interface. This further
   complicated the interface to the NRF24L01.
3. In a design such as this the power consumption of the CPU is only a part of
   the total picture - power usage by external peripherals needs to be managed
   correctly as well. One enhancement I intend to make to the design is to
   provide the processor sleep state available on a GPIO pin - this will allow
   external circuitry to shut down if needed when the processor goes into sleep
   mode which will further reduce the total power consumption.

Development on this prototype will continue with the eventual goal of moving to
a [EFM32HG108F64G](http://www.silabs.com/products/mcu/32-bit/efm32-happy-gecko/Pages/efm32-happy-gecko.aspx)
CPU on it's own dedicated board.

This prototype is part of a much larger project which is moving somewhat slowly
with only a single person working on it part time. If you find the design concept
interesting I encourage you to take a look at the [Sensaura](http://sensaura.org)
project in more detail - I would welcome any feedback or assistance.

