#ifndef HUSSAR_CORE_GEOMETRY_H
#define HUSSAR_CORE_GEOMETRY_H

#include <radar/radar.h>
#include <hussar/hussar.h>
#include <hussar/core/logging.h>
#include <cmath>
#include <Eigen/Core>

namespace hussar {

template<typename T, int N>
using Vector = Eigen::Matrix<T, N, 1>;

template<int N>
using VectorXf = Vector<Float, N>;

using Vector1f = VectorXf<1>;
using Vector2f = VectorXf<2>;
using Vector3f = VectorXf<3>;
using Vector4f = VectorXf<4>;

template<typename T>
using Vector3 = Vector<T, 3>;

using Vector2c = Vector<Complex, 2>;
using Vector3c = Vector<Complex, 3>;

using Matrix23c = Eigen::Matrix<Complex, 2, 3>;
using Matrix23f = Eigen::Matrix<Float, 2, 3>;
using Matrix32f = Eigen::Matrix<Float, 3, 2>;
using Matrix33f = Eigen::Matrix<Float, 3, 3>;
using Matrix44f = Eigen::Matrix<Float, 4, 4>;

#define AssertOrthogonal(a, b, msg) \
    Assert(std::abs(a.normalized().dot(b.normalized())) < 1e-4, msg)

#define AssertNormalized(a, msg) \
    Assert(std::abs(a.norm() - 1) < 1e-4, msg)

inline Float SurfaceAreaSphere(Float r) {
    return 4 * Pi * r * r;
}

inline Matrix32f buildFrame(const Vector3f &d) {
    Matrix32f result;
    if (std::abs(d.x()) > std::abs(d.y()))
        result.col(0) = Vector3f(-d.z(), 0, d.x()) / std::sqrt(d.x() * d.x() + d.z() * d.z());
    else
        result.col(0) = Vector3f(0, d.z(), -d.y()) / std::sqrt(d.y() * d.y() + d.z() * d.z());
    result.col(1) = d.cross(result.col(0));
    return result;
}

/**
 * @brief A ray is an infinitissimal element of a wave-front.
 * 
 * For each ray, we need to keep track of its polarization (i.e. phase, orientation and
 * strength), its origin and direction, the time it has been travelling so far
 * (to determine its phase at the receive), the frequency of the ray (since FMCW radars
 * emit a whole spectrum of frequencies) and its depth (i.e. how often it has been reflected
 * already).
 * 
 * @note Since we do not consider transmission in media in this project, we assume that the
 * ray always travels through free space (i.e. vacuum). For similar reasons, we only keep
 * track of the H field of the ray, since its E field can be uniquely derived from its
 * direction and its H field.
 */
class Ray {
public:
    HUSSAR_CPU_GPU Ray() {}
    HUSSAR_CPU_GPU Ray(const Vector3f &o) : o(o) {}
    HUSSAR_CPU_GPU Ray(const Vector3f &o, const Vector3f &d) : o(o), d(d) {
        AssertNormalized(d, "ray direction must be normalized");
    }

    HUSSAR_CPU_GPU Vector3f operator()(Float t) const {
        return o + t * d;
    }

    /// The origin of the ray.
    Vector3f o;
    
    /// The direction the ray is travelling in. Must be normalized.
    Vector3f d;

    /// The time the ray has been travelling so far (in [s]).
    Float time = 0;

    /// The frequency of the ray (in [Hz]).
    Float frequency;

    /// How many times this ray has been reflected in our integrator already.
    int depth = 0;

    /// The wave number of this ray.
    HUSSAR_CPU_GPU Float k0() const {
        return 2 * Pi * frequency / speed();
    }

    /// The wavelength of this ray.
    HUSSAR_CPU_GPU Float wavelength() const {
        return speed() / frequency;
    }

    /// The propagation speed of this ray, which we assume to always be the speed of light in vacuum (in [m/s]).
    HUSSAR_CPU_GPU Float speed() const { return radar::SPEED_OF_LIGHT; }

    /// Increments the time travelled by this ray by some distance (given in [m]).
    HUSSAR_CPU_GPU void addDistance(Float distance) { time += distance / speed(); }

    /// Sets the time travelled by this ray corresponding to some distance (given in [m]).
    HUSSAR_CPU_GPU void setDistance(Float distance) { time  = distance / speed(); }

    /// Reads the H field of this ray.
    HUSSAR_CPU_GPU Vector3c getH() const { return H; }

    /**
     * @brief Sets the H field of this ray.
     * 
     * @note This function will throw an error if the H field is not orthogonal to the propagation direction.
     */
    HUSSAR_CPU_GPU void setH(const Vector3c &v) {
        AssertOrthogonal(d, v, "H-field must be orthogonal to ray propagation direction");
        H = v;
    }

    /// Sets the H field of this ray to zero. This is convenient when we know the ray cannot reach the emitter.
    HUSSAR_CPU_GPU void setWeightToZero() { H = Vector3c::Zero(); }

    /// Multiplies the field strength by some scalar. Useful to take importance sampling weights into account.
    HUSSAR_CPU_GPU void weightBy(Float v) { H *= v; }

    /// Multiplies the field strength by some complex value. Useful to take scattering functions into account.
    HUSSAR_CPU_GPU void weightBy(Complex v) { H *= v; }

    /// Measures how strongly this ray would be received by e.g. an antenna.
    HUSSAR_CPU_GPU Complex measureH(const Vector3c &v) { return H.dot(v); }

protected:
    /// The H field associated with this ray.
    Vector3c H;
};

}

#endif
