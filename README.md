# ARM Cortex: _Compilation with GCC and GNU Make_

@author J. Alexander Gómez G.

@date 2020-Jun-20

## Overview

This repo enables any computer to build and flash an executable file for the NUCLEO-F413 board and can be ported to support almost any board or MCU that has an ARM Cortex-M CPU inside.   

This is part of what I have learned in courses [MCU1](), [Introduction to embedded software](),  [Embedded 2 coursera](), [Bare metal programming]() and refining it with some resources I mention at the end of this README and at the **Documentation** folder.

## Installation of the tools

The installation tutorial will be at the **Documentation** folder

## Content

This repo contains:

- _Makefile_ -- The elaborated mutiplatform makefile that builds and flash the executable
- _STM32F413ZHTX.lds_ -- Linker file
- _startup_stm32f413zhtx.c_ -- Startup file

The next files are included in my other Repo. 

- **src** folder. Contains the source files (.c) for the STM32
- **inc** folder. Contains the driver files (.h) for the STM32
- _main.c_ Main program to compile

## Usage

For using this application you have to move to the **_src_** directory.

Use: make \[TARGET]

### Targets

This **Makefile** can build 4 kinds of files:

- **\<FILE>.i** - Builds a \<FILE>.i Preprocessed file
- **\<FILE>.asm** - Dumps \<FILE>.asm Assembly file
- **\<FILE>.d** - Builds \<FILE>.d Dependency file
- **\<FILE>.o** - Builds a \<FILE>.o Object file

Example - This dumps the Assembly code of main.c for the STM32:

```
make main.asm
```

Other Targets:

- **all** - Builds all object files in project (links as well) and the output file
- **clean** - Removes all generated files

## References 

Some References checked for this repo:

- [Github Readme Basic writing and formatting syntax](https://help.github.com/en/github/writing-on-github/basic-writing-and-formatting-syntax)
- [GNU Make book through examples](https://makefiletutorial.com/)
- [GNU Make Cheat Sheet by bavo.van.achte](https://cheatography.com/bavo-van-achte/cheat-sheets/gnumake/)
- [GNU Make Cheat Sheet by eduardo lezcano](http://eduardolezcano.com/wp-content/uploads/2016/06/make_cheatsheet.pdf)

This is a repository where most of my work is shown. I wrote most part of the drivers and tried to explain the best to share this hard work in order to show comments and keep developing better embedded software./06/make_cheatsheet.pdf)

This is a repository where most of my work is shown. I wrote most part of the drivers and tried to explain the best to share this hard work in order to show comments and keep developing better embedded software.