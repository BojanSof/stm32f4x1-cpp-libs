# Embedded systems design and programming using STM32 MCU

## Learning roadmap

1. ARM overview

2. STM32 overview
    - Products overview.
    - Naming convention.
    - F4 family overview (our MCU's family).
    - ST-Link
    - ST-Link vs J-link

3. Setup envionment and build system
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

4. C/C++ and Git refresh
    - Only basic stuff
    - C++ advantages over C (price is greater conceptual complexity)
    - [Compiler Explorer](https://godbolt.org/), why is it useful, how and when to use it.

5. Writing simple `Makefile`s

6. `CMake` introduction
    - Writing simple `CMakeLists.txt`
    - How to use `CMake` from terminal.
    - How to use it in VS Code.

7. Setup STM32 CMake project template

8. MCU 'Hello World' using the project template
    - Explain the build process:
      - Compiling
      - Linking
    - Explain flashing process
    - Explain MCU target debugging

9. STM32 GPIO interface
  - Write C++ optimized GPIO interface
    - Do as much as possible in compile-time
    - Ideally, the C++ interface should bring 0 runtime overhead
  - Provides needed functionalities, like:
    - Easy to use Port notation, i.e. with `enum class`
    - Easy to use Pin notation
    - Setting pin mode, i.e. output, input, etc
    - Writing to pin
    - Reading from pin
  - `constexpr` keyword
