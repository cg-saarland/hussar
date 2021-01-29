# HUSSAR
HUSSAR ("hoch-effiziente and sensor-akkurate Radar Simulation") is a Radar simulation research project
funded by [RWTÜV Stiftung](https://www.rwtuev.de/en/shareholder/rwtuv-stiftung).

The goal of this project is to transfer ideas from Light Transport Simulation to Radar Simulation, with focus on:
* Accurate models suitable for automotive radar (FMCW Radar in 77 GHz band)
* Effective sampling strategies (such as low-discrepancy sampling, path guiding etc.)
* Efficient implementation (in particular using hardware accelerated ray tracing)

This repository contains the authors implementation that was developed as part of the project (in particular, [libhussar](libhussar)). A formal paper outlining and properly evaluating our algorithm is currently still in the works.

---

## Overview
The implementation is split up into several libraries and applications.  
Note that we do not currently provide any front-end for reading scene descriptions from files. Instead, we provide libraries and example code that can be used to easily write custom simulators that are tailored to the user's particular use-case. To this end, please have a look at the examples in the [`examples`](#examples) suite, which serve as a good starting point for writing custom simulators.

#### [libradar](libradar)
A common library that all other libraries and applications depend on. Provides fundamental data structures, physical units and constants, as well as equations for frequency-modulated continuous-wave (FMCW) Radar.

#### [libawrcapture](libawrcapture)
A utility that can communicate with Texas Instruments' `DCA1000EVM` and can parse the raw Radar signals streamed by it.  
This library is not required for simulation, but can be used to capture real world measurements from Texas Instruments' `AWR` sensor family for evaluation purposes.

#### [libhussar](libhussar)
The actual simulation library.  
This implements our Monte Carlo based Radar simulation algorithm, which can be executed both on the CPU as well as Nvidia GPUs.

#### [hussar2d](hussar2d)
An alternative simulation library and visualizer for two dimensional simulations.  
This application serves as playground for research ideas, which can often times be more easily verified and evaluated in a lower dimensional setting

#### [visualizer](visualizer)
Provides a graphical user interface to do experiments with both data capture (using `libawrcapture`) and simulation (using `libhussar`).
It also contains code to interface with the stepper motors attached to the Arduino in our measurement device.

#### [examples](examples)
Command-line applications that utilize `libhussar` to output simulated Radar signals for different test cases.  
These applications are a great starting point for anyone interested in using our libraries.

---

## Other resources
Apart from the implementation of our algorithm, this repository also includes some additional resources, in particular:

#### [cad](cad)
This directory contains models of our 3d printed parts that constitute our measurement setups.

#### [evaluation](evaluation)
Some of our measurements, in particular those of dihedral reflectors, can be found in this directory.
There is also a short evaluation and comparison with commercial software.
A full evaluation with more complex works and proper measurements is currently still in the works.

---

## Installation
This software is written entirely in C++ and uses CMake as build system.
It has been tested on Ubuntu 18 (with g++ 8) and macOS 10.15 (with clang 12). Windows is currently not yet supported.

### Dependencies
We require the following packages to be installed.

| Package | Example command using apt   |
|---------|-----------------------------|
| Eigen3  | `apt install libeigen3-dev` |

The following dependencies are optional, but recommended:

| Package | Purpose | Installation |
|---------|---------|--------------|
| fftw3   | FFT in visualizer | `apt install libfftw3-dev`  |
| glfw3   | visualizer        | `apt install libglfw3-dev`  |
| SDL2    | hussar2d          | `apt install libsdl2-dev`   |
| embree3 | Required for simulations on CPU | [Installation instructions](https://www.embree.org/downloads.html) |
| OptiX 7 | Required for simulations on GPU | [Installation instructions](https://developer.nvidia.com/designworks/optix/download) |

If you want to build this project with GPU support, please specify the path to your OptiX installation by passing `-DHUSSAR_OPTIX7_PATH=path/to/optix` to `cmake`.

Please note that some dependencies are fetched as git submodules. If you have pulled this repository without submodules, you can fetch them afterwards by running:
```bash
git submodule update --init --recursive
```

### Compiler
Make sure your compiler supports `#include <filesystem>`.
For GCC, this requires **g++-8** to be installed, which you can configure to be used in CMake by passing `-DCMAKE_CXX_COMPILER=g++-8` to `cmake`.

### Dropped packets on Linux
If you experience a problem with dropped packets (see `netstat -suna`),
then you need to raise the kernel receive buffer:

```
su -
echo 'net.core.rmem_default=16777216' >> /etc/sysctl.conf
echo 'net.core.rmem_max=16777216' >> /etc/sysctl.conf
sysctl --system # reload configuration
```
