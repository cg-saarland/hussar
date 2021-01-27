#ifndef HUSSAR_CORE_SAMPLER_H
#define HUSSAR_CORE_SAMPLER_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <vector>

namespace hussar {

class Sampler {
public:
    HUSSAR_CPU_GPU Sampler(long sampleCount)
    : sampleCount(sampleCount) {

    }

    HUSSAR_CPU_GPU virtual ~Sampler() {}

    /// Sets the index of the current sample.
    HUSSAR_CPU_GPU virtual void setSampleIndex(long index) = 0;

    /// Returns a uniformly distributed number in [0,1).
    HUSSAR_CPU_GPU virtual Float get1D() = 0;

    /// Returns a uniformly distributed vector in [0,1)^2.
    HUSSAR_CPU_GPU Vector2f get2D() {
        return Vector2f(get1D(), get1D());
    }

    /// Creates a copy of this sampler. Used for multi-threading.
    virtual Sampler *clone() = 0;

    /// Picks a random element from a vector with uniform probabilities.
    template<typename T>
    T &sampleVector(std::vector<T> &vector, Float *pdf = nullptr) {
        const size_t size = vector.size();
        assert(size > 0);
        if (pdf)
            *pdf /= size;

        Float rnd = get1D();
        size_t index = std::min(size_t(rnd * size), size - 1);
        return vector[index];
    }

    /// Number of samples, as used by SamplerIntegrator.
    long sampleCount;
};

}

#endif
