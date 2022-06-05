# Embedded systems design and programming using STM32 MCU

## Learning roadmap

1. Setup envionment and build system
    - Install cross-compiler and other tools for ARM (also known as toolchain, including `arm-none-eabi-gcc, arm-none-eabi-gdb, arm-none-eabi-newlib`)
      - What is cross-compiler?
      - Why is it needed?
      - ARM overview.
      - Why `none-eabi`?
    - Install `CMake`
      - What is `Makefile`?
      - Why use `Makefile`s?
      - What is `CMake`?
      - Why use `CMake` instead of `Makefile`s?
    - Install VS Code
      - Why not use manufacturer's IDE (i.e. STM32CubeIDE)?
      - Why VS Code?
    - Install useful extensions for VS Code
      - `C/C++` by Microsoft, which includes IntelliSense (code completition, paramter info, quick info, member list and other useful code info), debugging, etc.
      - `CMake Tools` by Microsoft for CMake integration in VS Code. 
      - `PlantUML` for drawing UML diagrams, useful for software design planning.

2. STM32 overview
    - Products overview.
    - Naming convention.
    - F4 family overview (our MCU's family).
    - ST-Link
    - ST-Link vs J-link

3. C/C++ and Git refresh
    - TODO
    - Only basic stuff
    - C++ advantages over C (price is greater conceptual complexity)
    - [Compiler Explorer](https://godbolt.org/), why is it useful, how and when to use it.

4. Writing simple `Makefile`s

5. `CMake` introduction
    - Writing simple `CMakeLists.txt`
    - How to use `CMake` from terminal.
    - How to use it in VS Code.
    - TODO: Add more detailed map here...

6. Setup STM32 CMake project template

7. MCU 'Hello World' using the project template
    - Explain the build process:
      - Compiling
      - Linking
    - Explain flashing process
    - Explain MCU target debugging

More to come...

## TODO:
  - Find resources for every topic
