#ifndef HUSSAR_CORE_HISTOGRAM_H
#define HUSSAR_CORE_HISTOGRAM_H

#include <vector>
#include <algorithm>

#include <hussar/hussar.h>
#include <hussar/core/image.h>

namespace hussar {

/// Assigns a weight to an element of arbitrary type by casting it to float if possible.
template<typename Element>
Float defaultWeightMapper(Element e) {
    return static_cast<Float>(e);
}

/**
 * @brief A histogram that can be used for radiation patterns that would be difficult to
 * describe and/or sample in analytical form.
 */
template<typename Element>
class Histogram : public Image<Element> {
public:
    Histogram(int width, int height) : Histogram(width, height,
        [](Element e) -> Float { return defaultWeightMapper<Element>(e); }
    ) {}

    template<typename WeightMapper>
    Histogram(int width, int height, WeightMapper&& wmap)
        : Image<Element>(width, height), m_rowAccum(width, height)
    {
        // Construct rebuild function to force inlining of weight mapping function
        m_rebuid = [this, wmap]() {
            Float rowAccum;
            Float totalAccum = 0;
            this->m_totalAccum.resize(this->height());
            for (int y = 0; y < this->height(); y++) {
                rowAccum = 0;
                for (int x = 0; x < this->width(); x++) {
                    rowAccum += wmap(this->at(x, y));
                    this->m_rowAccum.at(x, y) = rowAccum;
                }
                totalAccum += rowAccum;
                this->m_totalAccum[y] = totalAccum;
            }
            this->m_maxAccum = totalAccum;
        };
    }

    virtual ~Histogram() {}

    /**
     * @brief Return a sample using the provided random uniforms.
     * @note Assumes both elements of uv are strictly in range [0, 1].
     */
    Element sample(const Vector2f &uv) {
        // Determine the index of row to sample - O(lg(n))
        auto rowIt = std::lower_bound(m_totalAccum.begin(), m_totalAccum.end(), uv[1] * m_maxAccum);
        int rowIdx = rowIt - m_totalAccum.begin();
        // Compute the row value to sample
        Float maxRowValue = (rowIdx != 0 ? (m_totalAccum[rowIdx] - m_totalAccum[rowIdx-1]) : m_totalAccum[rowIdx]);
        Float sampleRowValue = uv[0] * maxRowValue;
        // Scan weight mapped image for correct entry - O(lg(n))
        int colIdx = -1;
        int b = 0;
        int maxColIdx = m_rowAccum.width() - 1;
        int t = maxColIdx;
        int m = t / 2; // intended integer truncation
        while (colIdx == -1 && b <= t) {
            if (sampleRowValue > m_rowAccum.at(m, rowIdx)) {
                // Value is in top half
                b = m + 1;
                // Check exit condition
                if (m == maxColIdx) {
                    // Element exceeds maximum in array; logic error!
                    break;
                } else if (sampleRowValue <= m_rowAccum.at(b, rowIdx)) {
                    colIdx = b;
                    break;
                }
            } else {
                // Value is in bottom half
                t = m - 1;
                // Check exit conditions
                if (m == 0) {
                    // Less than first element
                    colIdx = m;
                    break;
                } else if (sampleRowValue > m_rowAccum.at(t, rowIdx)) {
                    colIdx = t;
                    break;
                }
            }
            m = b + (t - b) / 2; // intended integer truncation
        }
        if (colIdx < 0) {
            throw std::logic_error("Column value not found");
        }
        return this->at(colIdx, rowIdx);
    }

    virtual void rebuild() {
        m_rebuid();
    }

private:
    std::function<void()> m_rebuid;
    // Accumulations along each row
    Image<Float> m_rowAccum;
    // Accumulation of row accumulation maximums
    std::vector<Float> m_totalAccum;
    Float m_maxAccum;
};

}

#endif
