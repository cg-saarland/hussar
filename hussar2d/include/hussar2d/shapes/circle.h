#ifndef HUSSAR_SHAPES_CIRCLE_H
#define HUSSAR_SHAPES_CIRCLE_H

#include <hussar2d/hussar2d.h>
#include <hussar2d/core/geometry.h>
#include <hussar2d/core/shape.h>

namespace hussar2d {

template<int Dimension>
class Hypersphere : public Shape {
private:
    using Vec = Vector<Float, Dimension>;

public:
    Hypersphere(
        const Vec &center,
        Float radius
    )
    : Shape(nullptr) {
        m_center = center;
        m_radius = radius;
        m_radiusSqr = radius*radius;
    }
    
    const Vec &center() const { return m_center; }
    Float radius() const { return m_radius; }

    void intersect(Intersection *isect) const {
        Float t = isect->ray.d.dot(m_center - isect->ray.o);
        
        Vec p = isect->ray(t);
        Vec n = p - m_center;

        Float sqrDist = n.squaredNorm();
        if (sqrDist > m_radiusSqr)
            return;
        
        Float dt = std::sqrt(m_radiusSqr - sqrDist);
        t -= dt;

        if (!isect->propose(t))
            return;
        
        isect->n = (isect->p - m_center) / m_radius;
        isect->material = material;
    }

private:
    Vec m_center;
    Float m_radius, m_radiusSqr;
};

using Circle = Hypersphere<2>;

}

#endif
