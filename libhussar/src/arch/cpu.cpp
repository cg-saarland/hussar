#include <hussar/arch/cpu.h>
#include <hussar/core/mesh.h>

#include <embree3/rtcore.h>

namespace hussar {
namespace cpu {

RTCRay rayFromIntersection(const Intersection &isect) {
    return RTCRay {
        isect.ray.o.x(), isect.ray.o.y(), isect.ray.o.z(),
        Epsilon, // tnear
        isect.ray.d.x(), isect.ray.d.y(), isect.ray.d.z(),
        0.f, // time
        isect.tMax, // tfar
        0, 0, 0
    };
}

RTCDevice getEmbreeDevice() {
    static bool EMBREE_INITIALIZED = false;
    static RTCDevice device;

    if (!EMBREE_INITIALIZED) {
        device = rtcNewDevice("");
        EMBREE_INITIALIZED = true;
    }
    
    return device;
}

Backend::RT::RT(const TriangleMesh &mesh) {
    m_scene = rtcNewScene(getEmbreeDevice());
    RTCGeometry geometry = rtcNewGeometry(getEmbreeDevice(), RTC_GEOMETRY_TYPE_TRIANGLE);

    void *v = rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(mesh.vertexBuffer[0]), mesh.vertexBuffer.size());
    memcpy(v, mesh.vertexBuffer.data(), sizeof(mesh.vertexBuffer[0]) * mesh.vertexBuffer.size());

    void *t = rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(mesh.indexBuffer[0]), mesh.indexBuffer.size());
    memcpy(t, mesh.indexBuffer.data(), sizeof(mesh.indexBuffer[0]) * mesh.indexBuffer.size());

    rtcCommitGeometry(geometry);
    rtcAttachGeometry(m_scene, geometry);
    rtcReleaseGeometry(geometry);

    rtcCommitScene(m_scene);
}

Backend::RT::~RT() {
    rtcReleaseScene(m_scene);
}

bool Backend::RT::visible(Intersection &isect) const {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    RTCRay ray = rayFromIntersection(isect);

    rtcOccluded1(m_scene, &context, &ray);
    
    return ray.tfar >= 0.f;
}

void Backend::RT::intersect(Intersection &isect) const {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRayHit rayhit;
    rayhit.ray = rayFromIntersection(isect);
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(m_scene, &context, &rayhit);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        isect.t = rayhit.ray.tfar;
        isect.p = isect.ray(isect.t);
        isect.n = Vector3f(
            rayhit.hit.Ng_x,
            rayhit.hit.Ng_y,
            rayhit.hit.Ng_z
        ).normalized();

        if (isect.n.dot(isect.ray.d) > 0) {
            // faceforward
            isect.n = -isect.n;
        }
    }
}

}
}
