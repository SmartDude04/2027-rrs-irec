#set page(
numbering: "1",
paper: "us-letter"
)
#set text(
font: "New Computer Modern",
size: 12pt
)
#set par(
justify: true,
)
#show link: set text(fill: blue)
#show link: underline

= Flashing to STM32

_Note_: These instructions are for Windows computers.

== Installing Software & Drivers

In order to properly detect the STM32, drivers must be installed. This can be done by installing #link("https://www.st.com/en/development-tools/stm32cubeprog.html")[STM32CubeProgrammer]. Follow the installation instructions in the executable.

== Plugging in the STM32

When plugged in, our STM32's (both F-series and H-series) start executing the code we flash onto it. In order to flash new code, the board must be put into `BOOT` mode. To do so, press the `BOOT` button on the board. To exit `BOOT` mode, press the `RST` button, and the code flashed to the chip will start executing again. Note that each time the chip powers on, it will start executing the code flashed onto it and thus the `BOOT` button must be pressed each time you wish to flash.

== Verifying Connection to the Computer

Once inside STM32CubeProgrammer, change the connection type to USB

#align(center)[
    #image("img/flashing-to-stm32_change-connection-type.png")
]

Refresh the port list and the `No DFU Detected` should turn into `USBX`, where `X` is some number. 

#align(center)[
    #image("img/flashing-to-stm32_refresh-ports.png")
]

Pressing `Connect` should connect the board. The table in the center of the screen is a representation of the code currently on the microcontroller. Congratulations, the STM32 is connecting to your computer! You can close STM32CubeProgrammer.

== Flashing with PlatformIO

Assuming a properly set-up `platformio.ini` file, code should be able to be compiled and flashed to the STM32. No other software can be connected while flashing is happening. That includes STM32CubeProgrammer, which may be open but not connected to the STM32.

== Extra Information

We are using Device Firmware Upgrade (DFU) mode to flash the firmware. This does not establish a COM port (a port that allows you to view output); you need to use a UART adapter to do that. Additionally, if you wish to debug, an ST-Link adapter is necessary (an ST-Link can also flash firmware).