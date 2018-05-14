# PinSim
XInput Game Controller for PC Pinball Games

Based on the excellent [MSF_FightStick XINPUT](https://github.com/zlittell/MSF-XINPUT) project by Zack "Reaper" Littell

######The complete project build guide, along with PCB links, wiring schematics, and 3D STL files, is [hosted at Tested](http://www.tested.com/tech/gaming/569647-how-build-pinsim-virtual-reality-pinball-machine/).

This code is designed to run on Teensy LC and requires the following libraries:

- [MSF-XInput](https://github.com/zlittell/MSF-XINPUT/tree/master/MSF_XINPUT)
- [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit ADXL345](https://github.com/adafruit/Adafruit_ADXL345)
- [Bounce](https://www.pjrc.com/teensy/td_libs_Bounce.html)
- [Average](https://github.com/MajenkoLibraries/Average)
- [EEPROMex](https://github.com/thijse/Arduino-EEPROMEx)

Please note the instructions for the MSF-XInput library, which requires some Teensyduino files to be overwritten in order to add support for a new USB device type.

I have included compiled code that can be installed on a Teensy LC using the [Teensy Loader](https://www.pjrc.com/teensy/loader.html) application. The code will compile for Teensy 3.2 but, besides being more expensive, that board lacks sufficient current output on the LED pins.

New feature added 5/13/2018:

* Hold Back and press Up on the joystick to map joystick to left analog stick. This is required by Oculus Arcade.
* Hold Back and press Down on the joystick to map joystick to D-pad (default).

New features & improvements added 6/20/2016:

* Pressing LB & A simultaneously sets plunger dead zone. This compensates for PC pinball games that have a huge plunger dead zone that works on Gamepads but is accentuated on real hardware. Just pull the plunger until just before the one in-game starts to move and then hit LB & A with your other hand. This setting is not saved between power cycles.
* The accelerometer is zeroed out every time A is pressed.
* Hold left flipper on boot to perform a rumble test.
* Hold right flipper on boot to disable accelerometer.
* Better plunge detection code that scales based on how far back the plunger is pulled.