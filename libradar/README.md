# libradar
This tiny header-only library contains data structures, physical units and constants, as well as equations for frequency-modulated continuous-wave (FMCW) Radar systems. Since it contains the basics required to represent and process Radar frames, it serves as common dependency for all other HUSSAR libraries and applications.

## Usage
@ todo

## Overview
Due to its simplicity, this library only consists of three files.

### `radar.h`
Contains data structures to represent the configuration and data of FMCW Radar frames. In particular

| Structure | Purpose |
|-----------|---------|
| `radar::RFConfig` | Specifies parameters such as frequency sweep slope, ADC rate, etc. of a FMCW Radar sensor |
| `radar::FrameConfig` | Specifies the dimensions of a Radar frame (how many chirps, samples per chirp, etc.) |
| `radar::Frame<Allocator>` | Stores the data of a Radar frame and allows simple access and processing operations |

Note that `radar::Frame` has an `Allocator` type argument. This is required to support allocation in shared CPU/GPU memory for applications that can run across devices. However, for simpler usages this argument can be left empty (i.e. `radar::Frame<>`) and it will fall back to the default `std::allocator` allocator.

### `units.h`
Include this to be able to use physical units in code, e.g.

```c++
#include <radar/units.h>

constexpr float ONE_GIGAHERTZ     = 1_GHz;
constexpr float TWO_NANOMETERS    = 2_nm;
constexpr float THREE_NANOSECONDS = 3_ns;
```

### `complex.h`
Since this library supports execution in the CUDA environment, a data type that can represent complex numbers across different devices is required. Since `std::complex` is not fully supported by CUDA, we resorted to implementing our own data type instead. This type is used by Radar frames to represent their data, and hence must be supported by users of this library.

## Installation
This library uses CMake as build system and requires a C++17 capable compiler. It can be used as a dependency of other libraries and applications by using the CMake `ADD_SUBDIRECTORY` command. This library has the following dependencies:

* Eigen3 (@todo this should actually be optional)
* fftw3 (optional, if available allows use of the `radar::Frame::fft` method)
