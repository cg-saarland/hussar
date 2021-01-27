#ifndef HUSSAR_ARCH_GPU_H
#define HUSSAR_ARCH_GPU_H

#include <hussar/hussar.h>
#include <hussar/core/frame.h>
#include <hussar/core/mesh.h>
#include <hussar/core/integrator.h>
#include <hussar/integrators/path.h> /// @todo hack

namespace hussar {
namespace gpu {

#ifdef HUSSAR_BUILD_GPU_RENDERER
/**
 * @brief The backend for running integrators on the GPU using OptiX for ray-tracing.
 */
struct Backend {
    template<typename Integrator>
    Backend(const TriangleMesh &mesh, Integrator &integrator)
    : m_state(mesh) {
        m_run = [&](const Scene &scene, long budget) {
            run(integrator, scene, budget); /// @todo
        };
    }

    void run(const Scene &scene, long budget, bool *interruptFlag = nullptr) {
        if (interruptFlag) {
            Log(EError, "task interruption is not supported by this backend");
        }

        m_run(scene, budget);
    }

private:
    void run(PathTracer &integrator, const Scene &scene, long budget);

    struct State {
        State(const TriangleMesh &mesh);
        State(const State &) = delete;
        ~State();

        void *data;
    };

    State m_state;
    std::function<void (const Scene &scene, long)> m_run;
};
#else
struct Backend {
    template<typename Integrator>
    Backend(const TriangleMesh &, Integrator &) {
        Log(EError, "GPU backend is not available as libhussar was compiled without OptiX");
    }

    void run(const Scene &, long, bool *) {}
};
#endif

}
}

#endif
