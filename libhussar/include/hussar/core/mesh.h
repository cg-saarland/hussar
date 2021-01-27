#ifndef HUSSAR_CORE_MESH_H
#define HUSSAR_CORE_MESH_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>

#include <vector>

namespace hussar {

/**
 * @brief Represents a collection of triangles used to describe the geometry of the scene.
 */
class TriangleMesh {
public:
    struct IndexTriplet {
        union {
            struct {
                int v0;
                int v1;
                int v2;
            };
            int raw[3];
        };
    };

    std::vector<Vector3f> vertexBuffer;
    std::vector<IndexTriplet> indexBuffer;

    void addQuad(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
        int i = (int)vertexBuffer.size();

        vertexBuffer.push_back(a);
        vertexBuffer.push_back(a + b);
        vertexBuffer.push_back(a + b + c);
        vertexBuffer.push_back(a + c);

        indexBuffer.push_back({{{ i+0, i+1, i+2 }}});
        indexBuffer.push_back({{{ i+0, i+2, i+3 }}});
    }

    void addBox(const Vector3f &min, const Vector3f &max) {
        Vector3f d;

        d = max - min;
        addQuad(min, Vector3f(0.f, d.y(), 0.f), Vector3f(0.f, 0.f, d.z()));
        addQuad(min, Vector3f(0.f, 0.f, d.z()), Vector3f(d.x(), 0.f, 0.f));
        addQuad(min, Vector3f(d.x(), 0.f, 0.f), Vector3f(0.f, d.y(), 0.f));

        d = min - max;
        addQuad(max, Vector3f(0.f, d.y(), 0.f), Vector3f(d.x(), 0.f, 0.f));
        addQuad(max, Vector3f(0.f, 0.f, d.z()), Vector3f(0.f, d.y(), 0.f));
        addQuad(max, Vector3f(d.x(), 0.f, 0.f), Vector3f(0.f, 0.f, d.z()));
    }
};

}

#endif
