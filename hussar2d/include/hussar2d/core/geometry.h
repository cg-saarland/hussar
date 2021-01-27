#ifndef HUSSAR2D_CORE_GEOMETRY_H
#define HUSSAR2D_CORE_GEOMETRY_H

#include <hussar2d/hussar2d.h>
#include <hussar2d/core/logging.h>
#include <radar/radar.h>
#include <cmath>
#include <Eigen/Core>

namespace hussar2d {

template<typename T, int N>
using Vector = Eigen::Matrix<T, N, 1>;

template<typename T, int N, int M>
using Matrix = Eigen::Matrix<T, N, M>;

using Vector1f = Vector<Float, 1>;
using Vector2f = Vector<Float, 2>;
using Vector3f = Vector<Float, 3>;

using Vector2c = Vector<Complex, 2>;
using Vector3c = Vector<Complex, 3>;

#define AssertOrthogonal(a, b, msg) \
    Assert(std::abs(a.normalized().dot(b.normalized())) < Epsilon, msg)

#define AssertNormalized(a, msg) \
    Assert(std::abs(a.norm() - 1) < Epsilon, msg)

inline Float SurfaceAreaSphere(Float r) {
    return 4 * Pi * r * r;
}

inline Vector3f expand(const Vector2f &v) {
    return Vector3f(v.x(), v.y(), 0.f);
}

class Ray {
public:
    Ray() {}
    Ray(const Vector2f &o) : o(o) {}
    Ray(const Vector2f &o, const Vector2f &d) : o(o), d(d) {
        AssertNormalized(d, "ray direction must be normalized");
    }

    Vector2f operator()(Float t) const {
        return o + t * d;
    }

    Vector2f o, d;
    Float time = 0;
    Float frequency;
    Float speed = radar::SPEED_OF_LIGHT;
    int depth = 0;

    Float k0() const { return 2 * Pi * frequency / speed; }
    void addDistance(Float distance) { time += distance / speed; }

    Vector3c getH() const { return H; }
    void setH(const Vector3c &v) {
        AssertOrthogonal(expand(d), v, "H-field must be orthogonal to ray propagation direction");
        H = v;
    }

    void setWeightToZero() { H = Vector3c::Zero(); }
    void weightBy(Float v) { H *= v; }

    Complex measureH(const Vector3c &v) const { return H.dot(v); }
    Complex waveValue() const {
        return std::exp(Complex(0, 2 * Pi * frequency * time));
    }

protected:
    Vector3c H;
};

}

#endif
