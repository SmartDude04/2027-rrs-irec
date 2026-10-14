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

= Compiling & Flashing

_Note_: The following instructions were only tested on Windows.

_Note_: PlatformIO must be installed to compile and flash code. To download platformio, see #link("https://platformio.org/")[this link].

== Using the Terminal

Because this repository is broken up into three separate PlatformIO projects, it may be easier to compile and flash the code via the Command Line Interface (CLI). First, open a terminal and `cd` into the directory (i.e. `airbrakes`, `payload`, or `ground-station`) you wish to build and flash from. 

To compile the code, run

```bash
platformio run
```

If successful, a `[`#text(fill: green)[`SUCCESS`]`]` line will appear at the bottom. If not, your code or `platformio.ini` file is not correct.

To compile and flash the code, run

```bash
platformio run -t upload
```

Remember that the STM32 must be in `BOOT` mode to receive new firmware, and that this must be entered into each time the microcontroller is powered back on if you wish to flash firmware.

== Using Visual Studio Code

Because this repository is broken up into three separate PlatformIO projects, to build with Visual Studio Code, you will need to leave the root project and open one of the sub folders. Once inside, open the PlatformIO extension window on the left tab. The `Run` and `Upload` buttons should be visible.

== Current Issue

Currently, a failure message appears when attempting to flash code to the STM32H7 at the end. It appears this is caused by the chip rebooting into the regular `RUN` mode after a successful firmware flash, but further testing needs to be done to confirm. In other words, the message can be ignored as the firmware was properly flashed but the computer can no longer communicate with the microcontroller. 