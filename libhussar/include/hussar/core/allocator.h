#ifndef HUSSAR_CORE_ALLOCATOR_H
#define HUSSAR_CORE_ALLOCATOR_H

#ifdef HUSSAR_BUILD_GPU_RENDERER
#include <cuda.h>
#include <cuda_runtime.h>
#endif

#include <hussar/hussar.h>

namespace hussar {

#ifdef HUSSAR_BUILD_GPU_RENDERER
/**
 * @brief Our custom allocator allows passing data between CPU and GPU more easily.
 */
template<typename T>
class Allocator {
public:
    using value_type = T;
    using reference = T &;
    using const_reference = const T &;

    Allocator() {}

    template<class U>
    Allocator(const Allocator<U> &) {}
  
    value_type *allocate(size_t n) {
        //std::cout << "allocate " << typeid(T).name() << " x" << n << " = " << (sizeof(T) * n) << " B" << std::endl;

        value_type *result = nullptr;
        cudaError_t error = cudaMallocManaged(&result, n * sizeof(T), cudaMemAttachGlobal);

        if (error != cudaSuccess) {
            throw std::bad_alloc();
        }

        return result;
    }
  
    void deallocate(value_type *ptr, size_t) {
        //std::cout << "deallocate " << typeid(T).name() << " x" << n << std::endl;

        cudaError_t error = cudaFree(ptr);

        if (error != cudaSuccess) {
            throw std::bad_alloc();
        }
    }
};

/// A reference is a managed pointer that can be shared.
template<class T1, class T2>
bool operator==(const Allocator<T1> &, const Allocator<T2> &) {
    return true;
}

template<class T1, class T2>
bool operator!=(const Allocator<T1> &lhs, const Allocator<T2> &rhs) {
    return !(lhs == rhs);
}
#else
template<typename T>
using Allocator = std::allocator<T>;
#endif

template<typename T>
using ref = std::shared_ptr<T>;

/// Convenience function to create ref objects easily.
template<typename T, class... Args>
ref<T> make_shared(Args&&... args) {
    return std::allocate_shared<T, Allocator<T>>({}, args...);
}

}

#endif
