# Embedded systems design and programming using STM32F4x1 MCU and C++

This repository contains peripherals library for STM32F4 BlackPill boards, which are based on STM32F401CC and STM32F411CE microcontrollers.
Besides the peripherals library, there are libraries for few external devices, such as OLED and TFT-LCD displays, touch controller, shift-registers, ultrasonic sensor and much more.
The code is written in C++, following the C++17 standard. The [Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) is used with [CMake](https://cmake.org/) for creating [Ninja](https://ninja-build.org/) build system.
[Visual Studio Code](https://code.visualstudio.com/) is the code editor of preference and debug configurations are included for it.
Currently, there is support for [JLink](https://www.segger.com/downloads/jlink/) and [ST-link V2](https://www.st.com/en/development-tools/st-link-v2.html) (including Chinese dongles) debuggers, which can be used for flashing and debugging the code via SWD.

The main purpose of this project is to learn how to use modern C++ in bare-metal programming.

## Prerequisites

- [Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
- [CMake](https://cmake.org/) (minimum version 3.18)
- [Ninja](https://ninja-build.org/)
- [Visual Studio Code](https://code.visualstudio.com/), with the following extensions:
  - [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
  - [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

For flashing and debugging, the following options are currently provided:
- [JLink](https://www.segger.com/downloads/jlink/)
- [ST-link V2](https://www.st.com/en/development-tools/st-link-v2.html). On Linux, [stlink](https://github.com/stlink-org/stlink) toolset can be used.
- `CMSIS-DAPLink` debug probe, with [OpenOCD](https://openocd.org/) as software interface for the probe. The probe used in the project can easily be bought from China, very cheaply. For Linux, `udev` rules file is provided under `templates/udev-rules` directory, which can be adjusted based on the USB VID and PID of the debug probe. Additional modifications on the `openocd.cfg` file may also be needed, found inside `cmake/stm32` directory.

The development was done on Linux, other platforms are not tested, but there shouldn't be difficulties when using other platforms.

## Current state of the project

STM32 peripherals:
- **GPIO** (manipulation and configuration, including alternate function names based on the peripherals, per pin, and external interrupts)
- **General purpose timers**
- **I2C master** asynchronous driver
- **SPI master** asynchronous driver
- **Serial USB**
- **Cycle counter**
- **Real time clock** (missing subsecond precision)

External devices:
- **SSD1306 OLED display**
- **24Cxxx Serial EEPROM**
- **74HC595 (SIPO) and 74HC165 (PISO) shift registers**
- **Two digit seven-segment display**
- **TFT LCDs** based on ILI9486(parallel) and ILI9488(SPI)
- **Resistive touch** based on XPT2046

There are few basic examples that are using the developed interfaces for testing purposes.

## Project structure

```
.
├── .vscode                       # debug configurations for VS Code
├── build                         # build artifacts
├── cmake                         # CMake related scripts
├── docs                          # currently contains useful drawings
├── lib                           # library dependencies for the examples
    ├── external-devices          # interfaces for devices like displays, sensors, etc.
    ├── stm32f4x1-devices         # interfaces for internal STM32F401/F411 peripherals
    └── usb                       # interfaces based on usb stack
├── reading-materials             # useful reading materials for C++, CMake, Embedded programming, ARM and STM32
├── src                           # source code for the example projects
├── svd                           # SVD files for debugging
├── templates                     # project templates
    ├── makefile-template         # makefile based project
    ├── project-template-cross    # STM32 CMake project template
    └── project-template-native   # CMake project template for PC
├── CMakeLists.txt                # Top-level CMakeLists.txt
├── CMakePresets.json             # Configuration and build CMake presets 
└── readme.md

```

## Future development

- Interfaces for other peripherals, including:
  - ADC
  - DMA
- Improving communication class hierarchy, including:
  - interface for half-duplex communication protocols
  - interface for full-duplex communication protocols
- Incorporating DMA in other peripheral drivers, like SPI, I2C, ADC, etc.
- Code optimizations and improvements for the peripherals drivers and external devices, most notably for TFT LCD drivers
- Proper testing of the code

## Resources and related projects

The following resources are great inspiration for this project:

1. [stm32-cmake](https://github.com/ObKo/stm32-cmake) - the CMake part for STM32 development is taken from this repo, with minor additions and adjustments
2. [EmbeddedGfx](https://github.com/nikodinovska/EmbeddedGfx) - graphics library developed and used for display interfaces
3. [Compile-time FSM generator](https://github.com/BojanSof/FSM-Generator) - C++17 compile-time FSM generator
4. [BareCpper](https://github.com/BareCpper/BareCpper) - great C++ bare-metal framework providing interfaces for multiple platforms
5. [TinyUSB](https://github.com/hathach/tinyusb) - open-source cross-platform USB host/device stack
6. [fetch_tinyusb](https://github.com/CMakePackageRegistry/fetch_tinyusb) - fetching tinyusb using CMake `FetchContent`