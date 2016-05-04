# PinSim
XInput Game Controller for PC Pinball Games

Based on the excellent [MSF_FightStick XINPUT](https://github.com/zlittell/MSF-XINPUT) project by Zack "Reaper" Littell

##The complete project build guide, along with PCB links, wiring schematics, and 3D STL files, is hosted at Tested.

This code is designed to run on Teensy LC and requires the following libraries:

- [MSF-XInput](https://github.com/zlittell/MSF-XINPUT/tree/master/MSF_XINPUT)
- [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit ADXL345](https://github.com/adafruit/Adafruit_ADXL345)
- [Bounce](https://www.pjrc.com/teensy/td_libs_Bounce.html)
- [Average](https://github.com/MajenkoLibraries/Average)
- [EEPROMex](https://github.com/thijse/Arduino-EEPROMEx)

Please note the instructions for the MSF-XInput library, which requires some Teensyduino files to be overwritten in order to add support for a new USB device type.

I have included compiled code that can be installed on a Teensy LC using the [Teensy Loader](https://www.pjrc.com/teensy/loader.html) application. The code will compile for Teenst 3.2 but that board, besides being more expensive, lacks sufficient current output on the LED pins.
