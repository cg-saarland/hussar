#ifndef HUSSAR_SHAPES_BOX_H
#define HUSSAR_SHAPES_BOX_H

#include <hussar2d/hussar2d.h>
#include <hussar2d/core/geometry.h>
#include <hussar2d/core/shape.h>

namespace hussar2d {

template<int Dimension>
class Hypercube : public Shape {
private:
    using Vec = Vector<Float, Dimension>;

public:
    Hypercube(
        const Vec &min,
        const Vec &max
    )
    : Shape(nullptr) {
        m_pos[0] = min;
        m_pos[1] = max;
    }
    
    const Vec &min() const { return m_pos[0]; }
    const Vec &max() const { return m_pos[1]; }

    void intersect(Intersection *isect) const {
        for (int d = 0; d < Dimension; ++d) {
            if (isect->ray.d[d] == 0)
                continue;
            
            Float side = m_pos[isect->ray.d[d] < 0][d];
            Float t = (side - isect->ray.o[d]) / isect->ray.d[d];
            if (!isect->willAcceptT(t))
                continue;
            
            auto p = isect->ray(t);
            for (int off = 1; off < Dimension; ++off) {
                int od = (d+off) % Dimension;
                if (p[od] < m_pos[0][od] || p[od] > m_pos[1][od])
                    goto no_intersection;
            }
            
            isect->t = t;
            isect->p = p;
            isect->n = Vec::Zero();
            isect->n[d] = 2 * (isect->ray.d[d] < 0) - 1;
            isect->material = material;
        
        no_intersection:
            continue;
        }
    }

private:
    Vec m_pos[2];
};

using Rectangle = Hypercube<2>;

}

#endif
