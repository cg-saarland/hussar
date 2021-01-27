#ifndef HUSSAR_CORE_HUSSAR_H
#define HUSSAR_CORE_HUSSAR_H

#if defined(HUSSAR_BUILD_GPU_RENDERER) && defined(__CUDACC__)
#define HUSSAR_CPU_GPU __host__ __device__
#define HUSSAR_GPU __device__
#else
#define HUSSAR_CPU_GPU
#define HUSSAR_GPU
#endif

#include <radar/complex.h>
#include <iostream>
#include <memory>
#include <complex>
#include <limits>
#include <cmath>

namespace hussar {

using Float = float;

using Complex = radar::complex<float>;

static constexpr Float Infinity = std::numeric_limits<Float>::infinity();
static constexpr Float Pi = 3.14159265358979323846;
static constexpr Float InvPi = 0.31830988618379067154;
static constexpr Float Inv2Pi = 0.15915494309189533577;
static constexpr Float Inv4Pi = 0.07957747154594766788;
static constexpr Float PiOver2 = 1.57079632679489661923;
static constexpr Float PiOver4 = 0.78539816339744830961;
static constexpr Float Sqrt2 = 1.41421356237309504880;
static const Float OneMinusEpsilon = std::nextafter(Float(1), Float(0));

// min. distance of objects: 0.05mm -- embree breaks somewhere around 1e-6!
static constexpr Float Epsilon = 5e-5;

class Antenna;
class Scene;
class Sampler;
class Intersection;
class Material;

}

#endif
