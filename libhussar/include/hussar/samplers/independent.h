#ifndef HUSSAR_SAMPLERS_INDEPENDENT_H
#define HUSSAR_SAMPLERS_INDEPENDENT_H

#include <hussar/hussar.h>
#include <hussar/core/sampler.h>
#include <hussar/core/random.h>

namespace hussar {

class IndependentSampler : public Sampler {
public:
    HUSSAR_CPU_GPU IndependentSampler(long sampleCount) : Sampler(sampleCount) {}

    HUSSAR_CPU_GPU void setSampleIndex(long index) {
        m_prng.setIndex(index);
    }

    HUSSAR_CPU_GPU Float get1D() {
        return m_prng();
    }

    virtual IndependentSampler *clone() {
        return new IndependentSampler(*this);
    }

private:
    PRNG m_prng;
};

}

#endif
