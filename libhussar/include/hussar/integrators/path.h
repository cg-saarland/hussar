#ifndef HUSSAR_INTEGRATORS_PATH_H
#define HUSSAR_INTEGRATORS_PATH_H

#include <hussar/hussar.h>
#include <hussar/core/emitter.h>
#include <hussar/core/integrator.h>
#include <hussar/core/sampler.h>
#include <hussar/core/guiding.h>
#include <hussar/core/allocator.h>

#include <hussar/samplers/halton.h> /// @todo hack!

#include <guiding/structures/btree.h>
#include <guiding/wrapper.h>

namespace hussar {

template<typename C, typename S = Float>
class GuidingWrapper {
public:
    typedef S Sample;
    typedef C Distribution;
    typedef typename Distribution::Vector Vector;
    typedef typename Distribution::AuxWrapper AuxWrapper;

    struct Settings {
        Float uniformProb = 0.5f;
        typename Distribution::Settings child;
    };

    Settings settings;
    std::function<void ()> onRebuild;

    GuidingWrapper() {
        reset();
    }

    GuidingWrapper(const Settings &settings)
    : settings(settings) {
        reset();
    }

    void operator=(const GuidingWrapper<S, C> &other) {
        settings   = other.settings;
        m_sampling = other.m_sampling;
        m_training = other.m_training;
    }

    void reset() {
        m_training = Distribution();
        m_sampling = Distribution();
    }

    template<typename ...Args>
    HUSSAR_CPU_GPU Float sample(Vector &x, Args&&... params) {
        if (settings.uniformProb == 1)
            return 1.f;
        
        Float pdf = 1 - settings.uniformProb; // guiding probability
        if (x[0] < settings.uniformProb) {
            x[0] /= settings.uniformProb;
            pdf *= m_sampling.pdf(
                settings.child,
                x,
                std::forward<Args>(params)...
            );
        } else {
            x[0] -= settings.uniformProb;
            x[0] /= 1 - settings.uniformProb;

            Float gpdf = 1;
            m_sampling.sample(
                settings.child,
                gpdf,
                x,
                std::forward<Args>(params)...
            );
            pdf *= gpdf;
        }

        pdf += settings.uniformProb;
        return pdf;
    }

    template<typename ...Args>
    HUSSAR_CPU_GPU Float pdf(Args&&... params) const {
        if (settings.uniformProb == 1)
            return 1.f;
        
        return settings.uniformProb + (1 - settings.uniformProb) * m_sampling.pdf(
            settings.child,
            std::forward<Args>(params)...
        );
    }

    template<typename ...Args>
    HUSSAR_CPU_GPU void splat(const Sample &sample, const AuxWrapper &aux, Float weight, Args&&... params) {
        //if (settings.uniformProb == 1)
        //    return;
        
        Float density = Float(sample);
        assert(std::isfinite(density));
        assert(density >= 0);
        assert(std::isfinite(weight));
        assert(weight >= 0);

        m_training.splat(
            settings.child,
            density, aux, weight,
            std::forward<Args>(params)...
        );
    }

    Distribution &training() { return m_training; }
    const Distribution &training() const { return m_training; }

    Distribution &sampling() { return m_sampling; }
    const Distribution &sampling() const { return m_sampling; }

    void step() {
        m_training.build(settings.child);
        m_sampling = m_training;
        m_training.refine(settings.child);

        if (onRebuild)
            onRebuild();
    }

private:
    Distribution m_sampling;
    Distribution m_training;
};

class PathTracer : public Integrator {
private:
    using GuidingTree = GuidingWrapper<
        guiding::BTree<2, guiding::Leaf<guiding::Empty>, guiding::Empty, Allocator>
    >;
    GuidingTree guiding;

    volatile Float currentSampleWeight;
    bool isFinalIteration;

    HUSSAR_CPU_GPU void stepGuiding() {
        guiding.step();
    }

    HUSSAR_CPU_GPU void clearFrame() {
        frame.clear();
        totalWeight = 0;
    }

public:
    bool onlyIndirect         = true; ///< direct path for FMCW not really of importance
    int maxDepth              = 10; ///< maximum number of GO bounces

    bool useGeometricalOptics = true;  ///< should always be true!
    bool poDiffraction        = !true; ///< ignore visibility for next event estimation
    bool doGuiding            = true;  ///< improves sampling significantly
    bool clearBeforeIteration = true; ///< requests that each render interation in guiding starts with a fresh frame

    bool correctPhase         = !true; ///< use with filteringSphere to simulate SBR behavior
    bool filterGuiding        = true; ///< helps with small filtering spheres

    bool doFiltering          = true; ///< similar to texture filtering
    bool filteringSphere      = true; ///< use sphere-like filtering instead of using differentials.
    Float filteringMin        = 600; ///< when filteringSphere = false
    Float filteringMax        = 900; ///< when filteringSphere = false

    Float filteringRadius     = correctPhase ? 0.5 : 160; ///< in wavelengths, used when filteringSphere = true

    template<typename Backend>
    void run(Backend &backend, const Scene &scene, long samples, bool *interruptFlag = nullptr) {
        setup();
        clearFrame();

        sampleIndexOffset = 0;
        currentSampleWeight = 1.f;

        if (!doGuiding) {
            backend.run(scene, samples, interruptFlag);
            return;
        }

        guiding.reset();
        isFinalIteration = false;
        
        long milestone = 16384;
        long remainingSamples = samples;

        while (true) {
            milestone = std::min(milestone, remainingSamples);
            //std::cout << "iteration (" << milestone << " samples)" << std::endl;

            backend.run(scene, milestone, interruptFlag);
            if (interruptFlag && *interruptFlag)
                return;
            
            sampleIndexOffset += milestone;

            remainingSamples -= milestone;
            if (remainingSamples == 0)
                break;
        
            milestone *= 2;

            if (remainingSamples < milestone * 2) {
                //std::cout << "  final iteration" << std::endl;
                isFinalIteration = true;
                milestone = remainingSamples;
            }

            if (clearBeforeIteration) {
                clearFrame();
                debug.clear();
                sampleIndexOffset = 0;
            } else {
                // new samples are giving more weight in our simulated frame, because they have
                // better quality (i.e. less noise) since guiding has been trained longer
                currentSampleWeight = 10 * currentSampleWeight;
            }

            stepGuiding();
        }
    }

    HUSSAR_CPU_GPU void setup() {
        Integrator::setup();

        guiding.settings.uniformProb = 0.1f;
        guiding.settings.child.splitThreshold = 0.005f;
        //guiding.settings.child.filtering = guiding::TreeFilter::EBox;
        guiding.settings.child.child.secondMoment = true;
    }

    template<typename RT>
    HUSSAR_CPU_GPU void sample(const Scene &scene, const RT &rt, long index) {
        HaltonSampler sampler;
        sampler.setSampleIndex(sampleIndexOffset + index);

        float maxDist = scene.rfConfig.adcRate / scene.rfConfig.freqSlope * radar::SPEED_OF_LIGHT; /// @todo not elegant

        Float sampleWeight = currentSampleWeight;

        Vector2f primary = Vector2f::Zero();
        Float primaryPdf = 1.f;

        Complex guidingWeight = 0;
        
        Intersection isect;
        Ray &ray = isect.ray;

        /// @todo there are multiple issues here:
        /// * the "real" bandwidth depends on the ADC-on-time, not the rampTime
        /// * frequency is time-dependent, but we do not model any time-dependency!
        ///   -> could use something like spectral rendering to take care of time.
        ///     -> that would definitely help with aliasing problems in Doppler as well!
        ray.frequency = scene.rfConfig.startFreq + sampler.get1D() * scene.rfConfig.bandwidth();

        SurfaceEmitter surface;

        Float r = 0;
        Float cosTheta = 1;

        while (true) {
            Vector3f tmpd = ray.d;

            if (ray.depth == 0) {
                primary = sampler.get2D();

                if (doGuiding) {
                    primaryPdf = guiding.sample(primary);
                }
                
                scene.tx.sample(primary, ray);
                ray.weightBy(1 / primaryPdf);
            } else {
                if (!useGeometricalOptics) {
                    Vector2f rnd = sampler.get2D();
                    surface.sample(rnd, ray);
                } else {
                    // geometrical optics

                    //ray.d = isect.R();
                    //emitter->evaluate(ray); // cos explicitly included

                    this->reflectRay(ray, isect); // cos not included, but included through hemisphere pdfs
                }
            }

            // MARK: - next event estimation
            /// @todo we only support single RX Radar at the moment
            for (size_t channel = 0; channel < 1; ++channel) {
                if (ray.depth == 0 && onlyIndirect)
                    break;
                
                Intersection nee = isect;
                nee.t = Infinity;
                
                Vector3c Hrx = scene.rx.nee(nee);
                if (!poDiffraction && !rt.visible(nee))
                    continue;

                nee.t = nee.tMax;
                if (ray.depth == 0) { /// @todo tagged dispatch
                    scene.tx.evaluate(nee.ray);
                    scene.tx.connect(nee);
                } else {
                    surface.evaluate(nee.ray);
                    surface.connect(nee);
                }
                Complex v = nee.ray.measureH(Hrx);

                Float dphase = 0;

                if (ray.depth > 0) {
                    if (correctPhase) {
                        Assert(useGeometricalOptics, "only geometrical optics supported for phase correction atm");
                        Assert(filteringSphere, "only sphere filtering supported for phase correction atm");

                        auto rxPos = nee.ray(nee.t);
                        auto virtualTx = nee.ray.o - r * ray.d;
                        auto dist = (virtualTx - rxPos).norm();
                        nee.ray.setDistance(dist);

                        v = ray.measureH(Hrx) * (Pi * dist) / Float(
                            std::pow(Pi * filteringRadius * nee.ray.wavelength(), 2)
                        ); // 1 / dist falloff
                        v /= 4 * Pi;
                        /// @todo cos(theta)?
                    } else if (useGeometricalOptics) {
                        // correct for the incorrect sampling density
                        // we sampled our last hitpoint with  'cos / r**2', but
                        // we really want '1 / (4*Pi*r)'
                        
                        v *= r / cosTheta;
                        v /= 4 * Pi;

                        if (cosTheta < 1e-3) {
                            continue;
                        }
                    }

                    if (filteringSphere) {
                        Assert(useGeometricalOptics, "only geometrical optics supported for sphere filtering atm");

                        auto rxPos = nee.ray(nee.t);
                        Float lambda = std::max<Float>(0, ray.d.dot(rxPos - ray.o));
                        Float rxDist = (ray(lambda) - rxPos).norm();
                        dphase = rxDist;
                    } else {
                        Float cos = (nee.ray.d - tmpd).normalized().dot(isect.n);
                        dphase = r * std::sqrt(1 - cos * cos) / cos;
                    }

                    dphase /= nee.ray.wavelength();

                    if (doFiltering) {
                        if (filteringSphere) {
                            if (dphase > filteringRadius) {
                                if (correctPhase) {
                                    // when we are correcting phase, we want a hard sphere.

                                    if (filterGuiding) {
                                        // we might want a soft sphere for guiding though
                                        guidingWeight += v / Float(std::pow(dphase / filteringRadius, 2) + 1);
                                    }

                                    v = 0;
                                } else {
                                    // use a soft sphere instead
                                    //v /= Float(std::pow(dphase / filteringRadius - 1, 2) + 1);
                                    v *= std::max<Float>(1 - 0.20 * (dphase / filteringRadius - 1), 0);
                                }
                            }
                        } else {
                            if (dphase > filteringMax) {
                                if (filterGuiding) {
                                    guidingWeight += v / Float(std::pow(dphase / filteringMax, 2) + 1);
                                }

                                v = 0;
                            }

                            if (dphase > filteringMin)
                                v *= (filteringMax - dphase) / (filteringMax - filteringMin);
                        }
                    }
                }
                
                //if (v == 0.f) continue;

                if (ray.depth > 0) {
                    guidingWeight += v;// * (dphase + Float(0.02f));
                }
                
                this->splat(scene, primary, ray.depth > 0 ? primaryPdf : 0, channel, nee.ray.time, dphase, v, sampleWeight);
            }
            
            // MARK: - random walk
            if (ray.depth >= maxDepth  || r >= maxDist) /// @todo hack!
                break;
            
            if (ray.getH().isZero(1e-20))
                break;
            
            // intersect with scene
            isect.reset();
            rt.intersect(isect);
            if (!isect.valid())
                // this ray has no master. this ray is a free ray!
                break;
            
            cosTheta = isect.cosTheta();
            if (cosTheta < 1e-3)
                // Grazing angles cause instabilities, since we would
                // divide by a low cosine value. We ignore these outliers. @todo
                break;
            
            r += isect.t;
            if (!useGeometricalOptics) {
                if (ray.depth == 0) { /// @todo tagged dispatch
                    scene.tx.connect(isect);
                } else {
                    surface.connect(isect);
                }
                ray.weightBy(isect.t * isect.t / cosTheta); // hemisphere pdf -> surface pdf
            } else {
                // geometrical optics
                /// @todo do we really need to multiply by 1j?
                //ray.weightBy(Complex(0, 1) / cosTheta);

                ray.addDistance(isect.t);
            }
            
            if (ray.getH().isZero(1e-20))
                break;
            
            // MARK: - prepare next bounce
            surface.incoming = isect;

            ray.o = isect.p;
            ray.depth++;
        }

        this->incrementTotalWeight(sampleWeight);
        this->splatDebug(primary, primaryPdf, sampleWeight);

        if (doGuiding && !isFinalIteration && primaryPdf > 0) {
            guiding.splat(std::abs(guidingWeight) * primaryPdf, {}, 1.f / primaryPdf, primary);
        }
    }

    HUSSAR_CPU_GPU RadarFrame fetchFrame() {
        return this->frame / Float(totalWeight);
    }

protected:
    long sampleIndexOffset;

#ifdef __CUDACC__
    double totalWeight;
#else
    std::atomic<double> totalWeight;
#endif

    HUSSAR_CPU_GPU void incrementTotalWeight(Float sampleWeight) {
#ifdef __CUDACC__
        atomicAdd(&totalWeight, sampleWeight);
#else
        /// @todo this is not elegant
        // (and could be more efficient under GCC)
        double oldV = totalWeight;
        while (!totalWeight.compare_exchange_weak(oldV, oldV + sampleWeight));
#endif
    }
};

}

#endif
