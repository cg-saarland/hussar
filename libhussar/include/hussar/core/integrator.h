#ifndef HUSSAR_CORE_INTEGRATOR_H
#define HUSSAR_CORE_INTEGRATOR_H

#include <thread>
#include <mutex>
#include <atomic>

#include <hussar/hussar.h>
#include <hussar/core/frame.h>
#include <hussar/core/scene.h>
#include <hussar/core/sampler.h>
#include <hussar/core/image.h>
#include <hussar/core/thread.h>
#include <hussar/core/logging.h>

#ifndef __CUDACC__
#include <hussar/io/tev.h>
#include <hussar/io/exr.h>
#endif

namespace hussar {

class Integrator {
public:
    struct DebugElement {
        float distance;
        Complex contribution;
        float dphase;
        float invPdfs;
        float weight;
        
        HUSSAR_CPU_GPU DebugElement &operator*=(float s) {
            distance *= s;
            contribution *= s;
            dphase *= s;
            invPdfs *= s;
            weight *= s;
            return *this;
        }
        
        HUSSAR_CPU_GPU DebugElement &operator+=(const DebugElement &other) {
            distance += other.distance;
            contribution += other.contribution;
            dphase += other.dphase;
            invPdfs += other.invPdfs;
            weight += other.weight;
            return *this;
        }
        
        HUSSAR_CPU_GPU DebugElement operator*(float s) const {
            DebugElement d(*this);
            return d *= s;
        }
    };

    using DebugImage = Image<DebugElement>;

    bool produceDebugImage = false;

    HUSSAR_CPU_GPU void configureFrame(const radar::FrameConfig &config) {
        frame.configure(config);
    }

    HUSSAR_CPU_GPU void clearFrame() {
        frame.clear();
    }

#ifndef __CUDACC__
    DebugImage getDebugImage() {
        DebugImage result = debug;
        result.each([](DebugElement &el) {
            el.contribution *= 1e-5; /// @todo debugging
            if (el.invPdfs > Epsilon) {
                el.contribution /= el.invPdfs;
                el.dphase /= el.invPdfs;
                el.distance /= el.invPdfs;
                el.invPdfs = 1;
            }
        });
        return result;
    }

    void saveDebugImage(const std::string &path) {
        if (produceDebugImage) {
            Log(EDebug, "Saving debug image...");

            DebugImage image = getDebugImage();

            ExrSaveFile exr { path + ".exr" };
            exr.add(image);
            exr.save();

            TevStream tev { "HUSSAR" };
            tev.add(image);
            tev.stream();
        }
    }
#endif
    
protected:
    HUSSAR_CPU_GPU void setup() {
        debug.clear();
        frame.clear();
    }

    /// Accounts for the phase shift that occurs when down-mixing the delayed RF signal.
    HUSSAR_CPU_GPU Complex measureRay(Float delta_t, const radar::RFConfig &rf) const {
        Complex phase = std::exp(Complex(0, 2*Pi * (rf.startFreq - delta_t * rf.freqSlope / 2) * delta_t));
        return phase;
    }
    
    /// Records the contribution from a path from TX to RX in the frame buffer.
    HUSSAR_CPU_GPU void splat(
        const Scene &scene,
        const Vector2f &txDir, float txPdf,
        int channel,
        float delta_t, float dphase,
        const Complex &measurement,
        Float weight
    ) {
        if (!produceDebugImage) {
            if (weight == 0 || (measurement.real() == 0 || measurement.imag() == 0))
                return;
        }

        delta_t += scene.rfConfig.antennaDelay;

        RadarFrame::PIndex index;
        index.setTime(delta_t, scene.rfConfig, frame.config());
        index.setVelocity(0.f, scene.rfConfig, frame.config());
        index.channel = channel;
        
        Complex contribution = measurement * measureRay(delta_t, scene.rfConfig);
        //contribution *= std::exp(-0.05f * delta_t * radar::SPEED_OF_LIGHT); /// @todo hack: simulate attenuation
        frame.splat(index, weight * contribution);
        
        if (produceDebugImage) {
            if (txPdf > 0) {
                debug.splat(txDir, (DebugElement){
                    .distance     = weight * radar::SPEED_OF_LIGHT * delta_t / txPdf,
                    .contribution = weight * contribution, // has already been divided by txPdf
                    .dphase       = weight * dphase / txPdf,
                    .invPdfs      = 0,
                    .weight       = 0
                });
            }
        }
    }

    /// Splats additional information into the debug image.
    HUSSAR_CPU_GPU void splatDebug(const Vector2f &txDir, float txPdf, Float weight) {
        if (produceDebugImage) {
            debug.splat(txDir, (DebugElement){
                .distance     = 0,
                .contribution = 0,
                .dphase       = 0,
                .invPdfs      = weight / txPdf,
                .weight       = weight
            });
        }
    }

    /// Reflects a ray in a perfectly specular manner.
    HUSSAR_CPU_GPU void reflectRay(Ray &ray, const Intersection &isect) const {
        Vector3c H = ray.getH();
        ray.d = isect.R();
        ray.setH(2 * isect.n * isect.n.dot(H) - H);
    }
    
    RadarFrame frame;
    DebugImage debug = DebugImage(1536, 512);
};

}

#endif
