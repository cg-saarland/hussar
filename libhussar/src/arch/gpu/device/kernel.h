#ifndef HUSSAR_ARCH_GPU_INTERNAL_KERNEL_H
#define HUSSAR_ARCH_GPU_INTERNAL_KERNEL_H

#include <optix.h>
#include <cuda_runtime.h>

#include <hussar/core/mesh.h> /// @todo hack!
#include <hussar/core/scene.h> /// @todo hack!
#include <hussar/integrators/path.h> /// @todo hack!

namespace hussar {

enum RayType {
    RAY_TYPE_RADIANCE = 0,
    RAY_TYPE_OCCLUSION = 1,
    RAY_TYPE_COUNT
};

struct GPUParams {
    unsigned int width;
    unsigned int height;
    unsigned int offset;

    const hussar::Scene *d_scene;
    hussar::PathTracer *d_integrator; /// @todo hack!

    OptixTraversableHandle handle;
};

struct RayGenData {
};

struct MissData {
};

struct HitGroupData {
    Vector3f *vertices;
    TriangleMesh::IndexTriplet *indices;
};

}

#endif
