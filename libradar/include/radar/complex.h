#ifndef LIBRADAR_COMPLEX_H
#define LIBRADAR_COMPLEX_H

#ifdef __CUDACC__
#define RADAR_CPU_GPU __host__ __device__
#else
/**
 * @brief Allows functions and methods to be executed both on the CPU and the GPU.
 * @note Only defined when __CUDACC__ is set.
 */
#define RADAR_CPU_GPU
#endif

#include <cmath>

namespace radar {

/**
 * @brief Our own complex number datatype, mostly compatible with `std::complex<Scalar>`.
 * 
 * Since `std::complex` is not available in CUDA, we implement our own datatype.
 * This also has the advantage that we can get a reference to `real` and `imag`,
 * which is not possible in newer versions of stdlibc++.
 * Note that we provide overloads for common operations (such as `std::abs`,
 * `std::norm` and `std::exp`) as well as Eigen support.
 * 
 * @todo it might also be possible to use `thrust::complex` on the CPU side, but it is unclear
 * how compatible it is with `std::complex`. At the very least, we would probably still have to
 * overload common operations such as `std:abs` etc.
 */
template<typename Scalar>
struct complex {
    typedef Scalar value_type;

    RADAR_CPU_GPU constexpr complex(const Scalar &r = Scalar(), const Scalar &i = Scalar())
    : m_real(r), m_imag(i) {}

    template<typename Other>
    RADAR_CPU_GPU constexpr complex(const complex<Other> &other)
    : m_real(other.real()), m_imag(other.imag()) {}

    RADAR_CPU_GPU Scalar &real() { return m_real; }
    RADAR_CPU_GPU Scalar &imag() { return m_imag; }
    RADAR_CPU_GPU const Scalar &real() const { return m_real; }
    RADAR_CPU_GPU const Scalar &imag() const { return m_imag; }

    /// In-place scalar multiplication.
    RADAR_CPU_GPU complex &operator*=(const Scalar &s) {
        m_real *= s;
        m_imag *= s;
        return *this;
    }

    /// In-place complex multiplication.
    template<typename Other>
    RADAR_CPU_GPU complex &operator*=(const complex<Other> &z) {
        const Scalar r = m_real * z.real() - m_imag * z.imag();
        m_imag = m_real * z.imag() + m_imag * z.real();
        m_real = r;
        return *this;
    }

    /// In-place complex division.
    template<typename Other>
    RADAR_CPU_GPU complex &operator/=(const complex<Other> &z);

    /// In-place scalar division.
    template<typename Other>
    RADAR_CPU_GPU complex &operator/=(const Other &z) {
        m_real /= z;
        m_imag /= z;
        return *this;
    }

    /// In-place complex addition.
    template<typename Other>
    RADAR_CPU_GPU complex &operator+=(const complex<Other> &z) {
        m_real += z.real();
        m_imag += z.imag();
        return *this;
    }

    /// In-place scalar addition.
    RADAR_CPU_GPU complex &operator+=(const Scalar &z) {
        m_real += z;
        return *this;
    }

    /// In-place complex subtraction.
    template<typename Other>
    RADAR_CPU_GPU complex &operator-=(const complex<Other> &z) {
        m_real -= z.real();
        m_imag -= z.imag();
        return *this;
    }

    /// In-place scalar subtraction.
    RADAR_CPU_GPU complex &operator-=(const Scalar &z) {
        m_real -= z;
        return *this;
    }

private:
    /// The real component of the complex number.
    Scalar m_real;
    /// The imaginary component of the complex number.
    Scalar m_imag;
};

/// Complex-complex multiplication.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator*(const complex<Scalar> &a, const complex<Scalar> &b) {
    complex<Scalar> r = a;
    r *= b;
    return r;
}

/// Complex-scalar multiplication.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator*(const complex<Scalar> &a, const Scalar &b) {
    complex<Scalar> r = a;
    r *= b;
    return r;
}

/// Scalar-complex multiplication.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator*(const Scalar &a, const complex<Scalar> &b) {
    complex<Scalar> r = b;
    r *= a;
    return r;
}

/// Complex-complex division.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator/(const complex<Scalar> &a, const complex<Scalar> &b) {
    complex<Scalar> r = a;
    r /= b;
    return r;
}

/// Complex-scalar division.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator/(const complex<Scalar> &a, const Scalar &b) {
    complex<Scalar> r = a;
    r /= b;
    return r;
}

/// Complex-complex addition.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator+(const complex<Scalar> &a, const complex<Scalar> &b) {
    complex<Scalar> r = a;
    r += b;
    return r;
}

/// Complex-scalar addition.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator+(const complex<Scalar> &a, const Scalar &b) {
    complex<Scalar> r = a;
    r += b;
    return r;
}

/// Complex-complex subtraction.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator-(const complex<Scalar> &a, const complex<Scalar> &b) {
    complex<Scalar> r = a;
    r -= b;
    return r;
}

/// Complex-scalar subtraction.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> operator-(const complex<Scalar> &a, const Scalar &b) {
    complex<Scalar> r = a;
    r -= b;
    return r;
}

/// Computes complex coordinates from polar coordinates.
template<typename Scalar>
RADAR_CPU_GPU inline complex<Scalar> polar(const Scalar &magnitude, const Scalar &angle) {
    /// @todo simultaneous sincos?
    return complex<Scalar>(magnitude * cos(angle), magnitude * sin(angle));
}

}

namespace std {

/// Computes the complex conjugate of a complex number.
template<typename Scalar>
RADAR_CPU_GPU inline radar::complex<Scalar> conj(const radar::complex<Scalar> &z) {
    return radar::complex<Scalar>(z.real(), -z.imag());
}

/// Computes the phase angle of a complex number.
template<typename Scalar>
RADAR_CPU_GPU inline Scalar arg(const radar::complex<Scalar> &z) {
    return atan2(z.imag(), z.real());
}

/// Returns the real component of a complex vector.
template<typename Scalar>
RADAR_CPU_GPU inline Scalar real(const radar::complex<Scalar> &z) {
    return z.real();
}

/// Returns the imaginary component of a complex vector.
template<typename Scalar>
RADAR_CPU_GPU inline Scalar imag(const radar::complex<Scalar> &z) {
    return z.imag();
}

/**
 * @brief Compute the magnitude of a complex number.
 * 
 * This method avoids floating point exponent underflows / overflows
 * by normalizing the components before squaring them.
 * 
 * @note taken from GNU libstdc++
 */
template<typename Scalar>
RADAR_CPU_GPU inline Scalar abs(const radar::complex<Scalar> &z) {
    Scalar x = abs(z.real());
    Scalar y = abs(z.imag());

    const Scalar s = x < y ? y : x;
    if (s == Scalar())    // well ...
        return s;
    
    x /= s; 
    y /= s;
    return s * sqrt(x * x + y * y);
}

/// Computes the squared norm of a complex number.
template<typename Scalar>
RADAR_CPU_GPU inline Scalar norm(const radar::complex<Scalar> &z) {
    const Scalar x = z.real();
    const Scalar y = z.imag();
    return x * x + y * y;
}

/// Computes the complex exponential function.
template<typename Scalar>
RADAR_CPU_GPU inline radar::complex<Scalar> exp(const radar::complex<Scalar> &z) {
    return radar::polar(exp(z.real()), z.imag());
}

}

namespace radar {

template<typename Scalar>
template<typename Other>
RADAR_CPU_GPU complex<Scalar> &complex<Scalar>::operator/=(const complex<Other> &z) {
    const Scalar r = m_real * z.real() + m_imag * z.imag();
    const Scalar n = std::norm(z);
    m_imag = (m_imag * z.real() - m_real * z.imag()) / n;
    m_real = r / n;
    return *this;
}

}


#include <Eigen/Eigen>

/**
 * @brief Adds support for the `radar::complex` datatype to the Eigen library.
 */
namespace Eigen {

template<typename _Real> struct NumTraits<radar::complex<_Real>>
: GenericNumTraits<radar::complex<_Real>> {
    typedef _Real Real;
    typedef typename NumTraits<_Real>::Literal Literal;
    enum {
        IsComplex = 1,
        RequireInitialization = NumTraits<_Real>::RequireInitialization,
        ReadCost = 2 * NumTraits<_Real>::ReadCost,
        AddCost = 2 * NumTraits<Real>::AddCost,
        MulCost = 4 * NumTraits<Real>::MulCost + 2 * NumTraits<Real>::AddCost
    };

    RADAR_CPU_GPU constexpr
    static inline Real epsilon() { return NumTraits<Real>::epsilon(); }
    RADAR_CPU_GPU constexpr
    static inline Real dummy_precision() { return NumTraits<Real>::dummy_precision(); }
    RADAR_CPU_GPU constexpr
    static inline int digits10() { return NumTraits<Real>::digits10(); }
};

}

#endif
