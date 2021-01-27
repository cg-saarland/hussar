#ifndef HUSSAR_SAMPLERS_RANDOM_H
#define HUSSAR_SAMPLERS_RANDOM_H

#include <hussar/hussar.h>
#include <hussar/core/sampler.h>
#include <hussar/core/random.h>

#include <random>

namespace hussar {

class StratifiedSampler : public Sampler {
public:
    HUSSAR_CPU_GPU StratifiedSampler(int w, int h)
    : Sampler(w * h), m_width(w), m_height(h) {}
    
    HUSSAR_CPU_GPU void setSampleIndex(long sampleIndex) {
        m_prng.setIndex(sampleIndex);
        m_sampleIndex = sampleIndex % (m_width * m_height);
        m_dimension = 0;
    }

    HUSSAR_CPU_GPU Float get1D() {
        return m_prng();
    }

    HUSSAR_CPU_GPU Vector2f get2D() {
        Vector2f uv;
        uv.x() = ((m_sampleIndex % m_width) + get1D()) / m_width;
        uv.y() = ((m_sampleIndex / m_width) + get1D()) / m_height;
        ++m_dimension;
        return uv;
    }

    virtual StratifiedSampler *clone() {
        return new StratifiedSampler(*this);
    }

private:
    PRNG m_prng;

    int m_dimension = 0;
    int m_sampleIndex = 0;
    int m_width, m_height;
};

}

#endif
