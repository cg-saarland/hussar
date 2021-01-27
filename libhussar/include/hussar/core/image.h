#ifndef HUSSAR_CORE_IMAGE_H
#define HUSSAR_CORE_IMAGE_H

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <hussar/core/allocator.h>

namespace hussar {

/**
 * @brief An image represents a two-dimensional array of arbitrary data.
 */
template<typename Element>
class Image {    
public:
    typedef Element ElementType;

    Image() : m_width(0), m_height(0) {}

    Image(int width, int height) : m_width(width), m_height(height) {
        m_data.resize(width * height);
        
        m_strideX = 1;
        m_strideY = width;
    }

    /// Executes a callback with a reference to each element of this image.
    HUSSAR_CPU_GPU void each(std::function<void (Element &)> callback) {
        for (int x = 0; x < m_width; ++x)
            for (int y = 0; y < m_height; ++y)
                callback(at(x, y));
    }
    
    HUSSAR_CPU_GPU int width() const { return m_width; }
    HUSSAR_CPU_GPU int height() const { return m_height; }
    
    HUSSAR_CPU_GPU Element *data() { return m_data.data(); }
    
    HUSSAR_CPU_GPU bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < m_width && y < m_height;
    }

    HUSSAR_CPU_GPU inline Element &at(const Vector2f &p) {
        Assert(p.x() >= 0 && p.x() < 1, "x out of bounds");
        Assert(p.y() >= 0 && p.y() < 1, "y out of bounds");

        int fx = p.x() * m_width;
        int fy = p.y() * m_height;
        int ix = std::floor(fx);
        int iy = std::floor(fy);
        return at(ix, iy);
    }

    HUSSAR_CPU_GPU inline Element &at(int x, int y) {
        return data()[(x % m_width) * m_strideX + (y % m_height) * m_strideY];
    }
    
    HUSSAR_CPU_GPU inline Element &evaluate(const Vector2f &p) {
        int ix = std::floor(p.x() * m_width);
        int iy = std::floor(p.y() * m_height);
        return at(ix, iy);
    }

    HUSSAR_CPU_GPU void splat(const Vector2f &p, const Element &e) {
        at(p) += e;
    }
    
    HUSSAR_CPU_GPU void clear(Element filler = Element()) {
        for (int x = 0; x < m_width; ++x)
            for (int y = 0; y < m_height; ++y)
                at(x, y) = filler;
    }

    HUSSAR_CPU_GPU virtual void rebuild() {}

private:
    std::vector<Element, Allocator<Element>> m_data;
    int m_width, m_height;
    int m_strideX, m_strideY;
};

}

#endif
