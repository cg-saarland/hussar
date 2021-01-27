#pragma diag_suppress 3126
#define RT_FUNCTION static __forceinline__ __device__

#include "kernel.h"
#include <hussar/hussar.h>

#include <optix_device.h>
#include <utility>

using namespace hussar;

#include "utils.h"
#include "polyfills.h"
#include "vecmath.h"

#define log(...)
//#define log(...) printf(__VA_ARGS__)

extern "C" {
  __constant__ GPUParams params;
}

struct RT {
    bool visible(Intersection &isect) const {
        unsigned int visible = 1u;

        optixTrace(
            params.handle,
            vec3_to_float3(isect.ray.o),
            vec3_to_float3(isect.ray.d),
            Epsilon,
            isect.tMax,
            0.0f,                    // rayTime
            OptixVisibilityMask(1),
            OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
            RAY_TYPE_OCCLUSION,      // SBT offset
            RAY_TYPE_COUNT,          // SBT stride
            RAY_TYPE_OCCLUSION,      // missSBTIndex
            visible
        );
        
        return visible;
    }

    void intersect(Intersection &isect) const {
        unsigned int u0, u1;
        packPointer(&isect, u0, u1);

        optixTrace(
            params.handle,
            vec3_to_float3(isect.ray.o),
            vec3_to_float3(isect.ray.d),
            Epsilon,
            isect.tMax,
            0.0f,                     // rayTime
            OptixVisibilityMask(1),
            OPTIX_RAY_FLAG_NONE,
            RAY_TYPE_RADIANCE,        // SBT offset
            RAY_TYPE_COUNT,           // SBT stride
            RAY_TYPE_RADIANCE,        // missSBTIndex
            u0, u1
        );
    }
};

extern "C" __global__ void __raygen__rg() {
    const uint3 optixIndex = optixGetLaunchIndex();
    const long sampleIndex = optixIndex.y * params.width + optixIndex.x + params.offset;

    const Scene &scene = *params.d_scene;
    PathTracer &integrator = *params.d_integrator;

    integrator.sample(scene, RT {}, sampleIndex);
}

extern "C" __global__ void __closesthit__radiance() {
  log("  closesthit_radiance\n");

  HitGroupData *rt_data = (HitGroupData *)optixGetSbtDataPointer();

  const TriangleMesh::IndexTriplet &indices = rt_data->indices[optixGetPrimitiveIndex()];
  const float3 v0 = vec3_to_float3(rt_data->vertices[indices.v0]);
  const float3 v1 = vec3_to_float3(rt_data->vertices[indices.v1]);
  const float3 v2 = vec3_to_float3(rt_data->vertices[indices.v2]);

  float3 normal = normalize(cross(v1 - v0, v2 - v0));

  // transform normal to world coordinates
  // float4 worldToObject[3];
  // optix_impl::optixGetWorldToObjectTransformMatrix(worldToObject[0], worldToObject[1], worldToObject[2]);
  // normal = normalize(optix_impl::optixTransformNormal(worldToObject[0], worldToObject[1], worldToObject[2], normal));
  
  // write intersection data
  Intersection &isect = getIsect();
  isect.t = optixGetRayTmax();
  isect.p = float3_to_vec3(optixGetWorldRayOrigin() + optixGetRayTmax() * optixGetWorldRayDirection());
  isect.n = float3_to_vec3(faceforward(normal, -optixGetWorldRayDirection(), normal));
}

extern "C" __global__ void __miss__radiance() {
    log("  miss_radiance\n");
    // nothing to do
}

extern "C" __global__ void __closesthit__occlusion() {
    optixSetPayload_0(0u); // set visible payload to 0
}
