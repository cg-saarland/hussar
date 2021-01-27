#ifndef HUSSAR2D_CORE_SHAPE_H
#define HUSSAR2D_CORE_SHAPE_H

#include <hussar2d/hussar2d.h>
#include <hussar2d/core/geometry.h>
#include <vector>

#include <Eigen/Geometry>

namespace hussar2d {

class Intersection {
public:
    Intersection(Ray &ray)
    : ray(ray) {}

    Float t = Infinity, tMax = Infinity;
    Vector2f p, n;
    Ray &ray;
    const Material *material;

    Vector2f wi() const { return -ray.d; }
    Vector2f R() const { return 2 * n.dot(wi()) * n - wi(); }
    Float cosTheta() const { return std::abs(n.dot(wi())); }
    Float cosThetaClamped() const { return std::max(n.dot(wi()), Float(0)); }
    bool valid() const { return t < Infinity; }

    void sampleMaterial(const Vector1f &uv, Ray &inc) const;
    void evaluateMaterial(Ray &inc) const;

    /// Queries whether a new intersection is closer than the current intersection.
    inline bool willAcceptT(Float newT) {
        return newT > Epsilon && newT < t && newT < tMax-Epsilon;
    }

    /**
     * @brief Suggests replacing the current closest hit with a new one.
     * @returns `true` if `newT` is indeed a closer intersection, `false` otherwise
     */
    inline bool propose(Float newT) {
        if (!willAcceptT(newT))
            return false;
        
        t = newT;
        p = ray(newT);

        return true;
    }
};

class Shape {
public:
    Shape(const Material *material)
    : material(material) {
    }

    virtual ~Shape() {}

    virtual void intersect(Intersection *isect) const = 0;

    const Material *material;
};

class AggregateShape : public Shape {
public:
    AggregateShape()
    : Shape(nullptr) {
    }
    
    void intersect(Intersection *isect) const {
        for (const auto &shape : shapes)
            shape->intersect(isect);
    }

    std::vector<std::shared_ptr<Shape>> shapes;
};

inline AggregateShape &operator<<(AggregateShape &a, Shape *s) {
    a.shapes.push_back(std::shared_ptr<Shape>(s));
    return a;
}

}

#endif
