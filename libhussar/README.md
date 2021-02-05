# libhussar
This library contains the author's implementation of their Monte Carlo based Radar simulation algorithm.
This document gives an overview on the structure of the implementation, how to compile it, as well as a brief example on how to use it.
A detailed description of the mathematical foundation of the algorithm will be made available shortly.

### Features
* Simulates frequency modulated continuous wave (FMCW) Radar systems
    * Outputs raw data captured by the sensor (i.e. the Radar cube)
    * Diffraction supported through Physical Optics model
* Uses Monte Carlo integration with variance reduction techniques from Light Transport Simulation
    * Guiding approaches allow learned importance sampling
    * Texture filtering avoids regions that interfere only destructively
    * Low-discrepancy sequences allow lower variance estimation of oscillatory integrals
* High-performance implementation that can run on different devices
    * Supports highly parallelized execution on CPUs
    * Supports specialized ray tracing hardware on Nvidia RTX GPUs
    * Without loss of code readability
* Supports geometries of arbitrary complexity
    * Thanks to the acceleration structures used by the ray tracing engines, this library can easily handle scenes with millions of triangles

### Limitations
* We only support triangle meshes in `*.obj` format
    * Support for other file formats must be added by the user
    * Intersection with curves (from CAD models) is not yet possible
* Diffraction is currently limited to a single diffraction event
* We only support perfect electric conductors (PEC) at the moment
* Movements of objects are not yet supported

## Overview
The underlying structure of this library is inspired by [PBRT](http://www.pbr-book.org), with support for GPU in particular inspired by [pbrt-v4](https://github.com/mmp/pbrt-v4).
The following table will give a short summary of the most important terms used throughout this implementation.

| Term | Meaning |
|------|---------|
| Backend | Responsible for scheduling and executing an integrator on a device (either CPU or GPU) |
| Integrator | The actual algorithm that performs the simulation |
| Frame | A FMCW Radar frame (also known as Radar cube) |
| Sampler | Produces a stream of (pseudo-)random numbers or low-discrepancy series for Quasi Monte Carlo |
| Scene | A collection of receive and transmit antennas, as well as the configuration of the Radar backend |
| TriangleMesh | The actual geometry that will be simulated, represented as a set of triangles |
| Emitter | A source of radiation (either directly through antennas or indirectly due to scattering) |
| Ray | An infinitissimal element of a wave front, which serve as the foundation of our simulation |
| Intersection | The result of a ray tracing operation, which contains the hit point along other useful information |

More detailed descriptions of these terms can be found in their corresponding code documentation.

### GPU backend
Instead of the wavefront-like approach of pbrt-v4, we have opted for a simpler to maintain megakernel GPU execution model.
While this allows us to provide a more readable reference implementation of our algorithm, please note that this also slightly hurts its performance.
Hence, we advise against this strategy in an efficiency-focused production setting.

**Warning:** we currently do not support tagged dispatch, which would be required for runtime polymorphism on GPUs. Put simply, if you want to use a different antenna model, radiation pattern, etc., you will have to make changes to this library and re-compile it. We are planning on adding tagged dispatch support later, but can not yet promise a release date for this feature.

## Example usage
This is a minimalistic example on how to use this library to perform simulations of FMCW Radar systems. For more complete examples, please take a look at the [`examples`](../examples) folder.

```c++
// configure the parameters of the FMCW ramps
radar::RFConfig rf;

rf.antennaDelay = 0.43_ns;

rf.startFreq = 77_GHz;
rf.freqSlope = 60_MHz / 1_us;
rf.adcRate   = 5_MHz;
rf.idleTime  = 100_us;
rf.rampTime  = 60_us;

// configure the parameters of captured frames
radar::FrameConfig frameConfig;
frameConfig.chirpCount      = 128;
frameConfig.samplesPerChirp = 256;
frameConfig.channelCount    = 4;

// geometry of the scene
hussar::TriangleMesh mesh;
hussar::WavefrontFile obj("scene.obj");
obj.read(mesh);

// simulation
auto scene = hussar::make_shared<hussar::Scene>();
scene->rfConfig = rf;

auto integrator = hussar::make_shared<hussar::PathTracer>();
integrator->configureFrame(frameConfig);

// run on the CPU
hussar::cpu::Backend backend { mesh, *integrator };

Matrix33f facing;
facing <<
    0, 0, -1,
    0, -1, 0,
    -1, 0, 0;

// place the antennas
scene->rx = NFAntenna {
    Vector3f(896_mm, 67_mm, -5_mm), // location
    facing,                         // local coordinate system
    AWRAngularDistribution()        // radiation pattern
};
scene->tx = NFAntenna {
    Vector3f(896_mm, 67_mm, -7_mm), // location
    facing,                         // local coordinate system
    AWRAngularDistribution()        // radiation pattern
};

// run the simulation
long sampleCount = 200*1000;
integrator->run(backend, *scene, sampleCount);
auto simulatedFrame = integrator->fetchFrame();

// do something with the simulation result here
// ...
```

## Installation
This software is written entirely in C++ and uses CMake as build system.
It has been tested on Ubuntu 18 (with g++ 8) and macOS 10.15 (with clang 12), but should run flawlessly on other platforms as well (e.g. Windows).

### Dependencies
We require the following packages to be installed.

| Package | Example command using apt   |
|---------|-----------------------------|
| Eigen3  | `apt install libeigen3-dev` |

Even though this library dependends on `libradar`, there is no dependency on `fftw3`, as no FFTs are performed in `libhussar`.

Depending on which device you want to run your simulations on, you will also need a ray tracing engine for it:

| Package | Purpose | Installation |
|---------|---------|--------------|
| embree3 | Required for simulations on CPU | [Installation instructions](https://www.embree.org/downloads.html) |
| OptiX 7 | Required for simulations on GPU | [Installation instructions](https://developer.nvidia.com/designworks/optix/download) |

If you want to build this project with GPU support, please specify the path to your OptiX installation by passing -DHUSSAR_OPTIX7_PATH=path/to/optix to cmake.

While it is possible to have both ray tracing engines installed at the same time, please note that simulations can still only run on a single device at a time. You can, however, run a separate simulation on the other device in the meanwhile.
