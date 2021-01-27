#ifndef HUSSAR_ARCH_CPU_H
#define HUSSAR_ARCH_CPU_H

#include <hussar/hussar.h>
#include <hussar/core/mesh.h>
#include <hussar/core/integrator.h>

typedef struct RTCSceneTy* RTCScene;

namespace hussar {
namespace cpu {

#ifdef HUSSAR_BUILD_CPU_RENDERER
/**
 * @brief The backend for running integrators on the CPU using Embree for ray-tracing.
 */
struct Backend {
    template<typename Integrator>
    Backend(const TriangleMesh &mesh, Integrator &integrator)
    : m_rt(mesh) {
        m_run = [&](const Scene &scene, long budget, bool *interruptFlag) {
            long sampleCount = 0;
            std::mutex scMutex;
            ThreadPool::get().parallel([&] (int) {
                while (!interruptFlag || !*interruptFlag) {
                    int batch;
                    long index;
                    {
                        std::unique_lock lock(scMutex);
                        batch = std::min<long>(budget - sampleCount, 256); 
                        if (batch <= 0)
                            break;
                        index = sampleCount;
                        sampleCount += batch;
                    }
                    
                    for (int j = 0; j < batch; ++j)
                        integrator.sample(scene, m_rt, index + j);
                }
            });
        };
    }

    void run(const Scene &scene, long budget, bool *interruptFlag = nullptr) {
        m_run(scene, budget, interruptFlag);
    }

private:
    struct RT {
        RT(const TriangleMesh &mesh);
        RT(const RT &) = delete;
        ~RT();

        bool visible(Intersection &isect) const;
        void intersect(Intersection &isect) const;

    private:
        RTCScene m_scene;
    };

    RT m_rt;
    std::function<void (const Scene &scene, long, bool *)> m_run;
};
#else
struct Backend {
    template<typename Integrator>
    Backend(const TriangleMesh &, Integrator &) {
        Log(EError, "CPU backend is not available as libhussar was compiled without embree");
    }

    void run(const Scene &, long, bool *) {}
};
#endif

}
}

#endif
