#ifndef LIBRADAR_RADAR_H
#define LIBRADAR_RADAR_H

#ifdef __CUDACC__
#define RADAR_CPU_GPU __host__ __device__
#else
/**
 * @brief Allows functions and methods to be executed both on the CPU and the GPU.
 * @note Only defined when __CUDACC__ is set.
 */
#define RADAR_CPU_GPU
#endif

#include <radar/complex.h>
#include <cstring>
#include <cassert>
#include <cstring>
#include <cmath>

#ifdef RADAR_HAS_FFTW3
#include <fftw3.h>
#endif

/**
 * @brief Contains data structures that allow describing radar backends.
 */
namespace radar {

/**
 * @brief The floating point precision to be used by libradar.
 * 
 * @warning This value can currently not be changed, since this would require
 * changes to the `Frame::fft` function (fftwf needs to be changed to fftw).
 */
typedef float Float;

/**
 * @brief The complex datatype to be used by libradar.
 */
typedef radar::complex<Float> Complex;

#ifndef __CUDACC__

namespace {
    /// Raw bit_cast functionality required by atomicAdd on x86.
    template <class To, class From>
    To bit_cast(const From &src) {
        To dst;
        // believe it or not, but compilers are clever enough to
        // optimize the memcpy away.
        std::memcpy(&dst, &src, sizeof(To));
        return dst;
    }
}

/**
 * @brief Atomically adds some value to a float.
 * 
 * This function has the same API as CUDA's `atomicAdd` functionality
 * (i.e. no `std::atomic<float>` is required!)
 */
static inline void atomicAdd(float *addr, float v) {
    static_assert(
        // first up, lets make sure your platform makes sense...
        sizeof(float) == sizeof(uint32_t) &&
        // ...and that atomics for uint32 do not require a mutex or something
        sizeof(uint32_t) == sizeof(std::atomic<uint32_t>)
        /// @todo is this really all that is required to guarantee that atomics are lock free?
    );

    // we will just pretend your float is an std::atomic.
    // why an std::atomic<uint32_t> and not an std::atomic<float> you ask?
    // because gcc cannot optimize std::atomic<float> very well!
    // (this trick is borrowed from pbrt v4)
    auto &bits = *(std::atomic<uint32_t> *)addr;

    uint32_t oldBits = bits, newBits;
    do {
        // you might think these bitcasts cost something, but that's not the case!
        newBits = bit_cast<uint32_t>(bit_cast<float>(oldBits) + v);
    } while (!bits.compare_exchange_weak(oldBits, newBits));
}

#endif

namespace {

/// Modulo operation that always returns positive values.
template<typename T>
RADAR_CPU_GPU T safe_modulo(T a, unsigned b) {
    T mod = a % b;
    if (mod < 0)
        mod += b;
    return mod;
}

/// Modulo one operation that always returns positive values.
template<typename Float>
RADAR_CPU_GPU Float modulo_one(Float v) {
    return v - std::floor(v);
}

}

/// The speed of light in free space (in [m/s]).
constexpr float SPEED_OF_LIGHT = 299792458;

/**
 * @brief Represents the characteristics of a frequency sweep for FMCW/CW radar systems.
 * @note In order to describe CW radar systems, you can set `freqSlope` to zero.
 */
struct RFConfig {
    /// The frequency at the start of the sweep (in [Hz])
    Float startFreq;
    /// The rate of change of the frequency (in [Hz/s])
    Float freqSlope;
    /// The sample rate of the raw data (in [Hz])
    Float adcRate;
    /// The idle duration between chirps (in [s])
    Float idleTime;
    /// The active duration of the sweep (in [s])
    Float rampTime;
    /// Additional round-trip delay by feed-lines, mixer etc. (in [s])
    Float antennaDelay;

    /// The bandwidth of a chirp (in [Hz])
    RADAR_CPU_GPU Float bandwidth() const {
        return freqSlope * rampTime;
    }
    
    /// Calculates how many chirps happen per second (in [Hz])
    RADAR_CPU_GPU Float chirpFrequency() const {
        return 1.f / (idleTime + rampTime);
    }
};

/**
 * @brief Represents the configuration of a radar frame (also known as radar cube).
 */
struct FrameConfig {
    static constexpr int NUM_COMPONENTS = 3;

    union {
        struct {
            /// How many chirps there are in a frame.
            int chirpCount;
            /// How many samples there are per chirp.
            int samplesPerChirp;
            /// How many RX channels there are in the frame.
            int channelCount;
        };

        /// @note order matches that of Frame::Index
        int raw[NUM_COMPONENTS];
    };

    size_t sampleCount() const {
        size_t count = 1;
        for (int i = 0; i < NUM_COMPONENTS; ++i)
            count *= raw[i];
        return count;
    }
};

/**
 * @brief Represents the data of a captured radar frame (also known as radar cube)
 * along with a description of its configuration.
 * 
 * @param Allocator The allocator used to allocate and release the data storage of the frame.
 * This is required to create radar frames that reside in CPU/GPU unified memory.
 */
template<typename Allocator = std::allocator<Complex>>
struct Frame {
    /**
     * @brief Denotes whether libradar has been compiled with FFT support.
     * @note libradar will be compiled with FFT support when fftw3 is available.
     */
    constexpr static bool SUPPORTS_FFT =
#ifdef RADAR_HAS_FFTW3
        true;
#else
        false;
#endif

    /// Denotes spaces that radar cubes can be defined in.
    enum Space {
        /// The radar cube is in unprocessed dimensions (the raw data captured by the radar sensor).
        /// Sample and chirp index in time dimension, channel index denotes antenna index.
        SPATIAL = 0,

        /// The radar cube is in frequency dimensions (typically used for processing and simulation).
        /// Sample index corresponds to distance, chirp index to velocity and channel index to incident angle.
        FOURIER = 1
    };

    /**
     * @brief Represents a point in the radar cube.
     * 
     * Note that this point does not need to have integer index coordinates.
     * When fractional coordinates are specified (i.e. the point lies between points defined by the grid
     * of the radar cube), it will be appropriately interpolated.
     * 
     * @note The interpolation used in `radar::Frame` assumes that a rectangular window function is used for
     * the fourier transform.
     */
    template<typename T>
    struct GenericIndex {
        static constexpr int NUM_COMPONENTS = 3;

        union {
            struct {
                /// The chirp index (corresponds to velocity in fourier space)
                T chirp;
                /// The sample index (corresponds to distance in fourier space)
                T sample;
                /// The channel index (corresponds to incident angle in fourier space)
                T channel;
            };
            T raw[NUM_COMPONENTS];
        };
        
        RADAR_CPU_GPU GenericIndex() {
            memset(raw, 0, sizeof(raw));
        }

        /**
         * @brief Sets the sample index so that it corresponds to the specified full roundtrip time.
         * 
         * @note This operation is only meaningful when working with frames that are in fourier space.
         */
        RADAR_CPU_GPU void setTime(Float delta_t, const RFConfig &rf, const FrameConfig &f) {
            if (rf.freqSlope == 0) {
                /// CW Radar (fixed frequency)
                sample = 0;
            } else {
                /// FMCW Radar (linear frequency sweep)
                Float delta_f = delta_t * rf.freqSlope; // in [Hz]
                sample = f.samplesPerChirp * modulo_one(delta_f / rf.adcRate);
            }
        }

        /**
         * @brief Sets the chirp index so that it corresponds to the specified velocity.
         * 
         * @note This operation is only meaningful when working with frames that are in fourier space.
         */
        RADAR_CPU_GPU void setVelocity(Float delta_v, const RFConfig &rf, const FrameConfig &f) {
            Float delta_p = 2 * rf.startFreq * delta_v / SPEED_OF_LIGHT;
            chirp = f.chirpCount * modulo_one(delta_p / rf.chirpFrequency());
        }
        
        /**
         * @brief Sets the sample index so that it corresponds to the specified full roundtrip distance
         * (typically this will be twice the distance of an object).
         * 
         * @warning More likely than not, you want to use `setTime` instead since this function
         * assumes that the wave only travelled through vacuum.
         */
        RADAR_CPU_GPU void setDistance(Float delta_s, const RFConfig &rf, const FrameConfig &f) {
            Float delta_t = delta_s / SPEED_OF_LIGHT; // in [s]
            setTime(delta_t, rf, f);
        }
        
        /**
         * @brief Computes the corresponding full roundtrip distance (typically this will be twice the
         * distance of an object) from the sample index assuming the wave travelled through vacuum only.
         */
        RADAR_CPU_GPU Float distance(const RFConfig &rf, const FrameConfig &f) const {
            Float delta_f = rf.adcRate * sample / Float(f.samplesPerChirp); // in Hz
            Float delta_t = delta_f / rf.freqSlope; // in s
            delta_t -= rf.antennaDelay;
            Float delta_s = SPEED_OF_LIGHT * delta_t; // full roundtrip in m
            return delta_s;
        }
        
        /// Computes the corresponding velocity from the chirp index.
        RADAR_CPU_GPU Float velocity(const RFConfig &rf, const FrameConfig &f) const {
            Float delta_p = rf.chirpFrequency() * nyquistBackfold(chirp, f.chirpCount);
            return delta_p * SPEED_OF_LIGHT / rf.startFreq / 2; /// @todo why startFreq?
        }

        /// Returns the closest grid point (nearest neighbor) in the radar cube.
        template<typename Int = int>
        RADAR_CPU_GPU GenericIndex<Int> rounded() const {
            GenericIndex<Int> result;
            for (int i = 0; i < NUM_COMPONENTS; ++i)
                result.raw[i] = std::round(raw[i]);
            return result;
        }
    
    private:
        /**
         * @brief Shifts a frequency from the range `[0,count]` into the range `[-count/2,+count/2]`
         * by folding back frequencies above the nyquist frequency `count/2`.
         * 
         * This is required when computing velocities from chirp indices, so that we do not incorrectly
         * view negative velocities as high positive velocities.
         */
        RADAR_CPU_GPU Float nyquistBackfold(Float i, int count) const {
            if (i > count / 2)
                i -= count;
            return i / count;
        }
    };
    
    /// Default representation of integral point in the radar cube (aligned with the radar cube grid).
    using Index = GenericIndex<int>;
    /// Default representation of fractional points in the radar cube (more precise for some computations).
    using PIndex = GenericIndex<Float>;
    
    RADAR_CPU_GPU Frame(const Allocator &alloc = {})
    : m_alloc(alloc) {}

    RADAR_CPU_GPU Frame(const Frame &frame, const Allocator &alloc = {})
    : m_alloc(alloc) {
        *this = frame;
    }

    RADAR_CPU_GPU Frame(Frame &&frame) {
        m_config = frame.m_config;
        m_space = frame.m_space;
        m_data = frame.m_data;
        m_fftPlan = frame.m_fftPlan;
        m_alloc = frame.m_alloc;

        frame.m_data = nullptr;
        frame.m_fftPlan = nullptr;
    }

    RADAR_CPU_GPU ~Frame() {
        freeData();
        destroyFFTPlan();
    }
    
    /**
     * @brief Sets all elements of this radar cube to zero.
     */
    RADAR_CPU_GPU void clear() {
        memset(m_data, 0, sizeof(Complex) * sampleCount());
    }

    /**
     * @brief Changes the dimensions of the radar cube described by this frame.
     * @warning This will erase all existing data.
     */
    RADAR_CPU_GPU void configure(const FrameConfig &config) {
        bool needsRealloc = config.sampleCount() != m_config.sampleCount();

        if (needsRealloc) {
            freeData();
            destroyFFTPlan();
        }

        m_config = config;
        m_data = m_alloc.allocate(sampleCount());
    }

    /**
     * @brief Performs an in-place component-wise addition of two frames.
     * 
     * @note The frames need to have equal configuration for this operation to be meaningful.
     */
    RADAR_CPU_GPU void operator+=(const Frame &other) {
        assert(sampleCount() == other.sampleCount());
        for (size_t i = 0, j = sampleCount(); i < j; ++i) {
            (*this)(i) += other(i);
        }
    }

    /**
     * @brief Performs an in-place component-wise scalar multiplication by a given factor.
     */
    RADAR_CPU_GPU void operator*=(Float f) {
        for (size_t i = 0, j = sampleCount(); i < j; ++i) {
            (*this)(i) *= f;
        }
    }
    
    /**
     * @brief Performs an in-place component-wise scalar division by a given denominator.
     *
     * This is useful for integrators that splat samples into the radar cube and need to normalize
     * it by total the amount of samples taken.
     */
    RADAR_CPU_GPU Frame operator/(Float f) const {
        Frame frame = *this;
        frame *= 1.f / f;
        return frame;
    }

    RADAR_CPU_GPU void operator=(const Frame &frame) {
        configure(frame.m_config);
        memcpy(m_data, frame.m_data, sizeof(Complex) * frame.sampleCount());

        m_space = frame.m_space;
        m_alloc = frame.m_alloc;
    }
    
    /// Returns the total amount of points in this radar cube.
    RADAR_CPU_GPU size_t sampleCount() const {
        return m_config.sampleCount();
    }

    RADAR_CPU_GPU FrameConfig config() const {
        return m_config;
    }

    /**
     * @brief Performs an in-place FFT operation with rectangular window function
     * on the radar cube.
     * 
     * This effectively flips the space of this frame (SPATIAL becomes FOURIER or
     * FOURIER becomes SPATIAL).
     * 
     * @todo This should probably also modify the `space` member of this frame.
     */
    RADAR_CPU_GPU void fft() {
#ifdef RADAR_HAS_FFTW3
        if (!m_fftPlan) {
            fftwf_plan_dft_3d(
                //channelCount, samplesPerChirp, chirpCount,
                m_config.chirpCount, m_config.samplesPerChirp, m_config.channelCount,
                (fftwf_complex *)m_data, (fftwf_complex *)m_data, // in-place
                FFTW_FORWARD, FFTW_ESTIMATE
            );
        }

        fftwf_execute((fftwf_plan)m_fftPlan);
#else
        assert(!"libradar has been compiled without FFT support!");
#endif
    }
    
    /**
     * @brief Returns the interpolated grid value at some point in the radar cube.
     * 
     * @note This assumes that a rectangular window function.
     */
    RADAR_CPU_GPU inline Complex operator()(const PIndex &idx) const {
        Index tmp;
        return interpolator<0>(idx, tmp);
    }
    
    /**
     * @brief Returns a reference to the grid value at some data index.
     * 
     * This is useful when enumerating all values in the radar cube, for example when
     * writing the frame to disk for later post processing and evaluation.
     */
    RADAR_CPU_GPU inline Complex &operator()(size_t idx) {
        return ((Complex *)m_data)[idx];
    }

    /**
     * @brief Returns a reference to the grid value at some grid-aligned point in the
     * radar cube.
     */
    RADAR_CPU_GPU inline Complex &operator()(const Index &idx) {
        return (*this)(makeIndex(idx));
    }

    /**
     * @brief Returns the grid value at some data index.
     * 
     * This is useful when enumerating all values in the radar cube, for example when
     * writing the frame to disk for later post processing and evaluation.
     */
    RADAR_CPU_GPU inline Complex operator()(size_t idx) const {
        return ((Complex *)m_data)[idx];
    }
    
    /**
     * @brief Returns the grid value at some grid-aligned point in the radar cube.
     */
    RADAR_CPU_GPU inline Complex operator()(const Index &idx) const {
        return (*this)(makeIndex(idx));
    }
    
    /**
     * @brief Returns the data index corresponding to some grid-aligned point in the radar cube.
     * 
     * @todo Rename this to makePoint?
     */
    RADAR_CPU_GPU size_t makeIndex(const Index &idx) const {
        size_t index = 0;
        for (int i = 0; i < idx.NUM_COMPONENTS; ++i) {
            index *= m_config.raw[i];
            index += idx.raw[i];
        }
        return index;
    }

    /**
     * @brief Computes the grid-aligned point at some data index in the radar cube.
     */
    RADAR_CPU_GPU Index makeIndex(size_t index) const {
        Index idx;
        for (int i = idx.NUM_COMPONENTS - 1; i >= 0; --i) {
            idx.raw[i] = index % m_config.raw[i];
            index /= m_config.raw[i];
        }
        return idx;
    }
    
    /**
     * @brief Performs naive frequency estimation at some grid-aligned point.
     * 
     * This can be useful to locating more precisely where a peak in the fourier
     * spectrum is.
     * 
     * @note Assumes a rectangular window function.
     * @todo Needs a better name.
     */
    RADAR_CPU_GPU PIndex frequencyEstimation(const Index &idx) const {
        // this might be suboptimal in the presence of noise
        
        size_t index = makeIndex(idx);
        PIndex p;

        int off = 1;
        for (int i = idx.NUM_COMPONENTS - 1; i >= 0; --i) {
            p.raw[i] = frequencyEstimation(index, off, idx.raw[i], m_config.raw[i]);
            off *= m_config.raw[i];
        }
        return p;
    }
    
    /**
     * @brief Returns the grid-aligned point with the highest grid value.
     */
    RADAR_CPU_GPU Index argmax() const {
        size_t i = 0;
        Float best = std::abs((*this)(i));
        for (size_t s = 1; s < sampleCount(); ++s) {
            Float c = std::abs((*this)(s));
            if (c > best) {
                i = s;
                best = c;
            }
        }
        
        return makeIndex(i);
    }

    /**
     * @brief Increments the value of some interpolated grid-cell by a specified amount.
     * 
     * For a point that happens to be grid-aligned, this corresponds to simply adding the
     * specified value to the grid value at that point. For non-grid-aligned points, however,
     * we need to simulate the effect of spectral leakage.
     * 
     * @note This assumes a rectangular window function.
     * 
     * @note Currently, this function only computes spectral leakage up to an index distance of
     * the `WindowSize` template parameter. Note that higher values are very costly (also due to
     * concurrent memory accesses), hence why we limit the range here.
     * 
     * @todo We should perform some approximation of the spectral leakage for the region outside of
     * this range.
     */
    template<int WindowSize = 16>
    RADAR_CPU_GPU void splat(const PIndex &index, Complex value) {
        Index center = index.rounded();
        Float shifts[PIndex::NUM_COMPONENTS];
        Float weight = 1.f;

        for (int i = 0; i < PIndex::NUM_COMPONENTS; ++i) {
            Float &shift = shifts[i];

            shift = index.raw[i] - center.raw[i];
            if (std::abs(shift) < 1e-4) {
                // shift is minuscule, essentially a delta peak in the Fourier domain
                shift = 0;
                continue;
            }
            
            Float shiftPi = M_PI * shift;
            value *= std::exp(Complex(0, shiftPi));
            weight *= std::sin(shiftPi) / M_PI;
        }

        splat<0, WindowSize>(center, value, shifts, weight);
    }

private:
    /**
     * @brief Helper function for splatting operations.
     */
    template<int index, int WindowSize>
    RADAR_CPU_GPU void splat(const Index &center, const Complex &value, const Float *shifts, Float weight) {
        if constexpr (index == Index::NUM_COMPONENTS) {
            // end of recursion
            //printf("splatting at %f+%fi at %lu\n", value.real(), value.imag(), makeIndex(center));

            /// @todo might be bad for performance
            Complex &v = (*this)(center);
            atomicAdd(&v.real(), weight * value.real());
            atomicAdd(&v.imag(), weight * value.imag());
            return;
        } else {
            // this else block is needed, otherwise template recursion is unbounded

            if (shifts[index] == 0) {
                // delta peak
                splat<index+1, WindowSize>(center, value, shifts, weight);
                return;
            }

            Index nextIndex = center;
            for (int shift = -WindowSize; shift <= +WindowSize; ++shift) {
                // splatting necessary
                /// @todo what about values that are farther away?

                nextIndex.raw[index] = safe_modulo(center.raw[index] + shift, m_config.raw[index]);
                splat<index+1, WindowSize>(
                    nextIndex,
                    value,
                    shifts,
                    weight / (shifts[index] - shift)
                );
            }
        }
    }

    /**
     * @brief Interpolation helper for frequency estimation.
     */
    RADAR_CPU_GPU Float frequencyEstimation(size_t idx, int off, int b, int max) const {
        Float l = std::abs((*this)(b > 0 ? idx - off : idx + (max-1) * off));
        Float m = std::abs((*this)(idx));
        Float r = std::abs((*this)(b < max-1 ? idx + off : idx - (max-1) * off));

        if ((l + m + r) == 0)
            return b;
        
        if (l > r)
            return b - l/(l+m);
        else
            return b + r/(r+m);
    }
    
    /**
     * @brief Interpolation helper for computing grid values for non-grid-aligned point.
     */
    template<int index>
    RADAR_CPU_GPU Complex interpolator(const PIndex &p, Index &idx) const {
        if constexpr (index == Index::NUM_COMPONENTS)
            return (*this)(idx);
        else {
            // this else block is needed, otherwise template recursion is unbounded

            idx.raw[index] = round(p.raw[index]);
            Float shift = p.raw[index] - idx.raw[index];
            idx.raw[index] = safe_modulo<int>(idx.raw[index], m_config.raw[index]);

            if (std::abs(shift) < 1e-4)
                return interpolator<index+1>(p, idx);

            Complex arg(0, 2 * M_PI * shift);
            return interpolator<index+1>(p, idx) * arg / (std::exp(arg) - Float(1.f));
        }
    }

    /**
     * @brief Releases the data storage used by this frame.
     */
    void freeData() {
        if (m_data) {
            m_alloc.deallocate(m_data, sampleCount());
            m_data = nullptr;
        }
    }

    /**
     * @brief Releases the FFT plan used by fftw3.
     * 
     * @note This functions has no effect if libradar is compiled without fftw3.
     */
    void destroyFFTPlan() {
#ifdef RADAR_HAS_FFTW3
        fftwf_free(m_data);
        fftwf_destroy_plan((fftwf_plan)m_fftPlan);
#endif
    }

private:
    /// Required by fftw3 to compute the FFT operation.
    void *m_fftPlan = nullptr;
    /// The raw data at the grid-aligned points in this radar cube.
    Complex *m_data = nullptr;

    /// The space that the data of this frame has to be interpreted in.
    Space m_space;
    /// The dimensions of the radar cube described by this frame.
    FrameConfig m_config;

    /// The allocator used to allocate and release the data storage of this radar frame.
    Allocator m_alloc;
};

}

#endif
