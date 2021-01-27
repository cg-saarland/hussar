#ifndef HUSSAR_CORE_RANDOM_H
#define HUSSAR_CORE_RANDOM_H

#include <hussar/hussar.h>
#include <random>

namespace hussar {

class PRNG {
public:
  PRNG(long seed = 31337) {
    m_seed = seed;
  }

  virtual void setIndex(long sample) {
    m_sample = sample;
    m_index = 0;
  }

  virtual Float operator()() {
    // This trick is borrowed from Mitsuba, which borrowed it from MTGP:
    // We generate a random number in [1,2) and subtract 1 from it.

    uint64_t tea = sampleTEA(m_index++);
    if constexpr (std::is_same<Float, float>::value) {
      union {
        uint32_t u;
        Float f;
      } x;

      x.u = ((tea & 0xFFFFFFFF) >> 9) | 0x3F800000U;
      return x.f - Float(1);
    } else if constexpr (std::is_same<Float, double>::value) {
      union {
        uint64_t u;
        Float f;
      } x;

      x.u = (tea >> 12) | 0x3FF0000000000000ULL;
      return x.f - Float(1);
    } else {
      assert(false);
    }
  }

private:
  uint64_t m_seed;
  uint64_t m_sample;
  uint32_t m_index;

  /**
   * @brief Generate fast and reasonably good pseudorandom numbers using the
   * Tiny Encryption Algorithm (TEA) by David Wheeler and Roger Needham.
   * 
   * For details, refer to "GPU Random Numbers via the Tiny Encryption Algorithm"
   * by Fahad Zafar, Marc Olano, and Aaron Curtis.
   * 
   * Implementation taken from Mitsuba 0.5.0.
   * @note we are using six rounds by default, because four tend to produce noticeable
   * artifacts.
   */
  inline uint64_t sampleTEA(uint32_t v1, int rounds = 6) {
    uint32_t sum = 0;
    uint32_t v0 = 0;

    uint32_t k0 = m_sample >> 32;
    uint32_t k1 = m_sample;
    uint32_t k2 = m_seed >> 32;
    uint32_t k3 = m_seed;

    for (int i = 0; i < rounds; ++i) {
      sum += 0x9E3779B9;
      v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
      v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    }

    return ((uint64_t) v1 << 32) + v0;
  }
};

}

#endif
