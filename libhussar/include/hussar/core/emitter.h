#ifndef HUSSAR_CORE_EMITTER_H
#define HUSSAR_CORE_EMITTER_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <hussar/core/sampler.h>
#include <hussar/core/sampling.h>
#include <hussar/core/intersection.h>
#include <cmath>
#include <array>

namespace hussar {

/**
 * @brief A source of radiation.
 * 
 * This can be an antenna or an infinitissimal surface patch that re-radiates (reflects)
 * radiation it received back into space.
 */
class Emitter {
public:
    HUSSAR_CPU_GPU virtual ~Emitter() {}

    /**
     * @brief Samples a ray emitted at this point.
     * 
     * Use this to continue paths in (adjoint) particle tracing.
     * This will set origin, direction and H field of the ray.
     */
    HUSSAR_CPU_GPU virtual void sample(const Vector2f &, Ray &) const = 0;

    /**
     * @brief Evaluates the radiation for a given point on the emitter and direction.
     * 
     * This will set the H field of the ray.
     */
    HUSSAR_CPU_GPU virtual void evaluate(Ray &) const = 0;

    /**
     * @brief Responsible for updating the ray after an intersection has been found.
     * 
     * This should increment the distance travelled by the ray and should multiply its
     * H field by the scattering characteristics of the object (e.g. 1/(4*Pi*r) falloff).
     */
    HUSSAR_CPU_GPU virtual void connect(Intersection &) const = 0;
};

/**
 * @brief Infinitissimal surface patch that re-radiates back into space due to surface currents.
 * 
 * @warning We only support PEC materials so far.
 */
class SurfaceEmitter {
public:
    Intersection incoming;

    HUSSAR_CPU_GPU void sample(const Vector2f &uv, Ray &ray) const {
        ray.d = UniformSampleSphere(uv);
        ray.weightBy(1 / UniformSpherePdf());
        
        evaluate(ray);
    }

    HUSSAR_CPU_GPU void evaluate(Ray &ray) const {
        Vector3c J = 2 * incoming.n.cross(incoming.ray.getH());
        ray.setH(ray.d.cross(J)); // cross product incorporates cosine term

        if (ray.d.dot(incoming.n) < 0)
            ray.setWeightToZero();
    }

    HUSSAR_CPU_GPU void connect(Intersection &outgoing) const {
        Float r = outgoing.t;
        outgoing.ray.addDistance(r);
        
        Complex green = (
            Complex(0, incoming.ray.k0())  // j*k
            + 1 / std::max(r, Float(1e-3)) // 1/r @todo
        ) / (4*Pi*r);
        outgoing.ray.weightBy(green);
    }
};

/**
 * @brief Represents the radiation pattern of an antenna.
 */
class AngularDistribution {
public:
    HUSSAR_CPU_GPU virtual ~AngularDistribution() {}
    
    /**
     * Samples a ray starting at this source.
     */
    HUSSAR_CPU_GPU virtual Vector3c sample(const Vector2f &uv, Vector3f &d) {
        d = UniformSampleSphere(uv);
        return evaluate(d) / pdf(d);
    }

    HUSSAR_CPU_GPU virtual Float pdf(const Vector3f &) {
        return UniformSpherePdf();
    }
    
    HUSSAR_CPU_GPU virtual Vector3c evaluate(const Vector3f &) = 0;
};

/**
 * @brief An approximation of the radiation pattern of the AWR1243 antennas.
 */
class AWRAngularDistribution {
public:
    HUSSAR_CPU_GPU AWRAngularDistribution() {}

    HUSSAR_CPU_GPU Vector3c sample(const Vector2f &uv, Vector3f &d) const {
        d = UniformSampleSphere(uv);
        return evaluate(d) / pdf(d);
    }

    HUSSAR_CPU_GPU Float pdf(const Vector3f &) const {
        return UniformSpherePdf();
    }

    /**
     * @note We do not have precise antenna measurements for our AWR device, so we resort to
     *       a rough approximation of the radiation pattern using simple polynomials.
     */
    HUSSAR_CPU_GPU Vector3c evaluate(const Vector3f &d) const {
        Vector3c H = Vector3c(0, 1, 0).cross(d);
        
        Float cos_h = std::sqrt(Float(1) - d.x() * d.x());
        Float cos_e = std::sqrt(Float(1) - d.y() * d.y());

        H *= 2.622 / std::pow(cos_h - 1.8, 6);
        H *= 0.625 / std::pow(cos_e - 1.5, 4);

        return H;
    }
};

class Antenna : public Emitter {
public:
    HUSSAR_CPU_GPU virtual void sample(const Vector2f &uv, Ray &ray) const = 0;
    HUSSAR_CPU_GPU virtual void evaluate(Ray &) const = 0;
    HUSSAR_CPU_GPU virtual void connect(Intersection &) const = 0;

    /**
     * @brief Constructs an intersection object that can be used to test visibility of the antenna.
     * 
     * @returns The H-field of the antenna in the direction of connection.
     */
    HUSSAR_CPU_GPU virtual Vector3c nee(Intersection &nee) const = 0;
};

/**
 * @brief Models an antenna with infinitissimal area that can be used to simulate radar sensors.
 * 
 * This is equivalent to a point light source in computer graphics.
 * The radiation pattern is determined by the provided AngularDistribution.
 */
class NFAntenna {
public:
    HUSSAR_CPU_GPU NFAntenna() {}
    HUSSAR_CPU_GPU NFAntenna(
        const Vector3f &position,
        const Matrix33f &rotation,
        const AWRAngularDistribution &radiation
    )
    : m_position(position), m_rotation(rotation), m_radiation(radiation) {
        Assert(
            std::abs(std::abs(m_rotation.determinant()) - 1) < 1e-2,
            "supplied rotation matrix is - in fact - not a rotation matrix!"
        );
    }

    HUSSAR_CPU_GPU void sample(const Vector2f &uv, Ray &ray) const {
        ray.o = m_position;

        Vector3c H = m_radiation.sample(uv, ray.d);
        ray.d = Vector3f(m_rotation * ray.d);
        ray.setH(m_rotation * H);
    }

    HUSSAR_CPU_GPU void evaluate(Ray &ray) const {
        ray.o = m_position;
        Vector3c H = m_radiation.evaluate(m_rotation.transpose() * ray.d);
        ray.setH(m_rotation * H);
    }

    HUSSAR_CPU_GPU void connect(Intersection &nee) const {
        Float r = nee.t;

        nee.ray.addDistance(r);
        nee.ray.weightBy(1 / (4*Pi*r));
    }

    HUSSAR_CPU_GPU Vector3c nee(Intersection &nee) const {
        nee.ray.d = m_position - nee.ray.o;
        Float r = nee.ray.d.norm();
        nee.ray.d /= r;
        nee.tMax = r;

        Vector3c H = m_radiation.evaluate(m_rotation.transpose() * (-nee.ray.d));
        return m_rotation * H;
    }

    HUSSAR_CPU_GPU Vector3f &position() { return m_position; }
    HUSSAR_CPU_GPU Matrix33f &rotation() { return m_rotation; }
    HUSSAR_CPU_GPU const Vector3f &position() const { return m_position; }
    HUSSAR_CPU_GPU const Matrix33f &rotation() const { return m_rotation; }

private:
    Vector3f m_position;
    Matrix33f m_rotation;
    AWRAngularDistribution m_radiation;
};

/**
 * @brief Models a far-field antenna that can be used for RCS measurements and to emit plane waves.
 * 
 * This is equivalent to a directional light in computer graphics.
 * Internally, this is represented by a disk of radius `radius` with distance `radius` to
 * the point `center`. The normal of this disk is `-dir`.
 * The radiation pattern is specular in direction `-dir`.
 * 
 * @todo Add geometry so this can be hit by material sampling?
 */
class FFAntenna : public Antenna {
public:
    HUSSAR_CPU_GPU FFAntenna(
        const Vector3f &dir,
        const Vector3c &polarization,
        const Vector3f &sceneCenter,
        Float radius
    ) : m_dir(dir), m_polarization(polarization), m_radius(radius) {
        AssertNormalized(dir, "normal of plane wave must be normalized");
        AssertOrthogonal(dir, polarization, "H-field must be orthogonal to direction of propagation");

        m_frame = buildFrame(-dir);
        m_center = sceneCenter + dir * radius;
    }

    HUSSAR_CPU_GPU virtual void sample(const Vector2f &uv, Ray &ray) const {
        Vector2f disk = 2 * uv - Vector2f::Ones(); // ConcentricSampleDisk(uv)
        ray.o = (m_frame * disk) * m_radius + m_center;
        ray.d = -m_dir;

        Float pdf = 1 / (4 * std::pow(m_radius, 2)); // ConcentricDiskPdf() / (radius*radius);
        ray.setH(m_polarization / pdf);
    }

    /**
     * @note It is impossible to hit this antenna, as its angular distribution is a dirac delta peak.
     */
    HUSSAR_CPU_GPU virtual void evaluate(Ray &ray) const {
        ray.setWeightToZero();
    }

    HUSSAR_CPU_GPU virtual void connect(Intersection &) const {
        // nothing to do here
    }

    HUSSAR_CPU_GPU virtual Vector3c nee(Intersection &nee) const {
        Vector3f local = nee.ray.o - m_center;
        local -= local.dot(m_dir) * m_dir;

        if (local.norm() > 1.f) {
            /// outside of disk
            return Vector3c::Zero();
        }

        nee.ray.d = m_dir;
        nee.tMax = (local + m_center - nee.ray.o).norm();

        return m_polarization * (4.f * Pi * nee.tMax);
    }

protected:
    /// The direction of propagation (i.e. normal of the plane wave).
    Vector3f m_dir;

    // Two vectors orthonormal to the direction, used for sampling.
    Matrix32f m_frame;

    /// The H field of this emitter. Must be orthogonal to the direction of propagation.
    Vector3c m_polarization;

    /// Center of the disk.
    Vector3f m_center;

    /// Radius of the disk. Should match the bounding sphere radius of the scene.
    Float m_radius;
};

}

#endif
