# Levitator Arduino Code

This repository hosts the Arduino code for managing an electromagnet levitator system, developed within the workshop curriculum at HTL Bulme.

## Overview

The levitator system employs a PID controller to regulate the position of an object levitated by an electromagnet. It communicates with a graphical user interface (GUI) via serial communication to update parameters such as setpoint, proportional, integral, and derivative gains.

## Requirements

- PlatformIO
- Arduino Nano (ATmega328)

## Installation

To utilize this code, proceed with the following steps:

1. Clone this repository to your local machine.
2. Open the project folder in PlatformIO.
3. Upload the code to your Arduino Nano.

## Usage

Connect the Arduino Nano to your electromagnet system and power it up. Ensure that the GUI from [PyLevit](https://github.com/x-vmaier/PyLevit) is operational and connected to the Arduino via serial communication.

The system will receive input from the GUI to update parameters and control the electromagnet accordingly.

## Implementation Details

### Packet Format/Serialization

Communication between the Arduino and the GUI is facilitated through packet-based serial communication. Each packet comprises a start delimiter, identifier, separator, data payload, and end delimiter, structured as `{identifier:data}`. The `fastprotoc` library handles packet serialization and deserialization.

### EEPROM

System parameters such as setpoint, proportional gain, integral gain, and derivative gain are stored in the EEPROM to retain their values across power cycles. The `EEPROMHandler` class manages reading and writing data to EEPROM, ensuring that only altered values are written.

## Credits

- **Author:** [Valentin Maier](https://github.com/x-vmaier)
- **Institution:** HTL Bulme

## License

This code is licensed under the [MIT License](LICENSE). Feel free to use, modify, or distribute it according to the terms of the license.

For more details, refer to the [PyLevit repository](https://github.com/x-vmaier/PyLevit) for integrating the levitator system with the graphical user interface.