#ifndef HUSSAR2D_HUSSAR2D_H
#define HUSSAR2D_HUSSAR2D_H

#include <iostream>
#include <memory>
#include <complex>
#include <limits>

namespace hussar2d {

using Float = float;
using Complex = std::complex<float>;

static constexpr Float Infinity = std::numeric_limits<Float>::infinity();
static constexpr Float Pi = 3.14159265358979323846;
static constexpr Float InvPi = 0.31830988618379067154;
static constexpr Float Inv2Pi = 0.15915494309189533577;
static constexpr Float Inv4Pi = 0.07957747154594766788;
static constexpr Float PiOver2 = 1.57079632679489661923;
static constexpr Float PiOver4 = 0.78539816339744830961;
static constexpr Float Sqrt2 = 1.41421356237309504880;
static constexpr Float Epsilon = 1e-4; // min. thickness of objects: 1um

class Antenna;
class Integrator;
class Scene;
class Sampler;
class Intersection;
class Material;

template<typename T, bool is_array = false>
class copy_ptr {
protected:
    T *ptr = nullptr;

public:
    copy_ptr() {}
    copy_ptr(T *ptr) : ptr(ptr) {
        //std::cout << "init to " << ptr << std::endl;
    }

    copy_ptr(const copy_ptr<T> &other) {
        *this = other;
    }

    copy_ptr(copy_ptr<T> &&other) noexcept {
        ptr = other.ptr;
        //std::cout << "move to " << other.ptr << std::endl;
        other.ptr = nullptr;
    }

    ~copy_ptr() noexcept {
        reset(nullptr);
    }

    T *get() const {
        return ptr;
    }

    void reset(T *newPtr) {
        //std::cout << "set to " << newPtr << std::endl;
        if (is_array)
            delete[] ptr;
        else
            delete ptr;
        ptr = newPtr;
    }

    T *operator->() { return ptr; }
    const T *operator->() const { return ptr; }

    void operator=(const copy_ptr &other) {
        if (!other.get())
            this->reset(nullptr);
        else if (!this->get())
            this->reset(new T(*other.get()));
        else
            *this->get() = *other.get();
    }
};

}

#endif
