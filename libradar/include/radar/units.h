#ifndef LIBRADAR_UNITS_H
#define LIBRADAR_UNITS_H

#include <complex>
#include <cstring>
#include <strings.h>

static constexpr long double operator ""_Hz (long double v) { return 1e+0 * v; }
static constexpr long double operator ""_kHz(long double v) { return 1e+3 * v; }
static constexpr long double operator ""_MHz(long double v) { return 1e+6 * v; }
static constexpr long double operator ""_GHz(long double v) { return 1e+9 * v; }
static constexpr long double operator ""_Hz (unsigned long long v) { return 1e+0 * v; }
static constexpr long double operator ""_kHz(unsigned long long v) { return 1e+3 * v; }
static constexpr long double operator ""_MHz(unsigned long long v) { return 1e+6 * v; }
static constexpr long double operator ""_GHz(unsigned long long v) { return 1e+9 * v; }

static constexpr long double operator ""_s (long double v) { return 1e-0 * v; }
static constexpr long double operator ""_ms(long double v) { return 1e-3 * v; }
static constexpr long double operator ""_us(long double v) { return 1e-6 * v; }
static constexpr long double operator ""_ns(long double v) { return 1e-9 * v; }
static constexpr long double operator ""_s (unsigned long long v) { return 1e-0 * v; }
static constexpr long double operator ""_ms(unsigned long long v) { return 1e-3 * v; }
static constexpr long double operator ""_us(unsigned long long v) { return 1e-6 * v; }
static constexpr long double operator ""_ns(unsigned long long v) { return 1e-9 * v; }

static constexpr long double operator ""_m (long double v) { return 1e-0 * v; }
static constexpr long double operator ""_dm(long double v) { return 1e-1 * v; }
static constexpr long double operator ""_cm(long double v) { return 1e-2 * v; }
static constexpr long double operator ""_mm(long double v) { return 1e-3 * v; }
static constexpr long double operator ""_um(long double v) { return 1e-6 * v; }
static constexpr long double operator ""_nm(long double v) { return 1e-9 * v; }
static constexpr long double operator ""_m (unsigned long long v) { return 1e-0 * v; }
static constexpr long double operator ""_dm(unsigned long long v) { return 1e-1 * v; }
static constexpr long double operator ""_cm(unsigned long long v) { return 1e-2 * v; }
static constexpr long double operator ""_mm(unsigned long long v) { return 1e-3 * v; }
static constexpr long double operator ""_um(unsigned long long v) { return 1e-6 * v; }
static constexpr long double operator ""_nm(unsigned long long v) { return 1e-9 * v; }

#endif
