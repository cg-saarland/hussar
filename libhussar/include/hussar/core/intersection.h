#ifndef HUSSAR_CORE_SHAPE_H
#define HUSSAR_CORE_SHAPE_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <vector>

#include <Eigen/Geometry>

namespace hussar {

/**
 * @brief Represents the intersection of a ray with a surface.
 * @see Ray
 */
class Intersection {
public:
    HUSSAR_CPU_GPU Intersection() {
        reset();
    }

    /// The distance to the hit point.
    Float t;

    /// The maximum distance to search for intersections.
    Float tMax;

    /// The position of the hit point.
    Vector3f p;

    /// The normal of the surface at the intersection.
    Vector3f n;

    /// The ray used for intersection.
    Ray ray;
    
    /// Returns the incident direction of the ray, pointing away from the intersection point.
    HUSSAR_CPU_GPU Vector3f wi() const { return -ray.d; }

    /// Returns the direction of perfect specular reflection.
    HUSSAR_CPU_GPU Vector3f R() const { return 2 * n.dot(wi()) * n - wi(); }

    /// Returns the cosine of the angle between normal and ray direction.
    HUSSAR_CPU_GPU Float cosTheta() const { return std::abs(Float(n.dot(wi()))); }

    /// Returns the cosine of the angle between normal and ray direction with negative values clamped to zero.
    HUSSAR_CPU_GPU Float cosThetaClamped() const { return std::max(Float(n.dot(wi())), Float(0)); }

    /// Returns whether an intersection closert than tMax has been found.
    HUSSAR_CPU_GPU bool valid() const { return t < tMax; }

    /// Resets the intersection.
    HUSSAR_CPU_GPU void reset() {
        t = Infinity;
        tMax = Infinity;
    }
};

}

#endif
