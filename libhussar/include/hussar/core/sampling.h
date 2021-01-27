/**
 * This file uses functions from PBRT.
 */

#ifndef HUSSAR_CORE_SAMPLING_H
#define HUSSAR_CORE_SAMPLING_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <cmath>

namespace hussar {

inline Vector2f ConcentricSampleDisk(const Vector2f &u) {
    // Map uniform random numbers to $[-1,1]^2$
    Vector2f uOffset = 2.f * u - Vector2f(1, 1);

    // Handle degeneracy at the origin
    if (uOffset.x() == 0 && uOffset.y() == 0)
        return Vector2f(0, 0);

    // Apply concentric mapping to point
    Float theta, r;
    if (std::abs(uOffset.x()) > std::abs(uOffset.y())) {
        r = uOffset.x();
        theta = PiOver4 * (uOffset.y() / uOffset.x());
    } else {
        r = uOffset.y();
        theta = PiOver2 - PiOver4 * (uOffset.x() / uOffset.y());
    }
    return r * Vector2f(std::cos(theta), std::sin(theta));
}

inline Float ConcentricDiskPdf() {
    return InvPi;
}

inline Vector3f UniformSampleSphere(const Vector2f &u) {
    Float phi = 2 * Pi * u[0];
    Float y = 2 * u[1] - 1;
    Float r = std::sqrt(std::max((Float)0, (Float)1 - y * y));
    return Vector3f(-r * std::sin(phi), y, -r * std::cos(phi));
}

inline Float UniformSpherePdf() {
    return Inv4Pi;
}

inline Vector3f CosineSampleHemisphere(const Vector2f &u) {
    Vector2f d = ConcentricSampleDisk(u);
    Float z = std::sqrt(std::max<Float>(0, 1 - d.x() * d.x() - d.y() * d.y()));
    return Vector3f(d.x(), d.y(), z);
}

inline Float CosineHemispherePdf(Float cosTheta) {
    return cosTheta * InvPi;
}

}

#endif
