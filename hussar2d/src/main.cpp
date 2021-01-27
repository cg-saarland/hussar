#include <hussar2d/shapes/rectangle.h>
#include <hussar2d/shapes/circle.h>

#include <iostream>
#include <random>
#include <thread>
#include <array>

namespace guiding {
    using Float = hussar2d::Float;

    template<int D>
    using VectorXf = std::array<Float, D>;
}

#include <guiding/wrapper.h>
#include <guiding/structures/btree.h>

#include "renderer.h"


using namespace std;
using namespace hussar2d;

class RandomSampler {
public:
    RandomSampler() {}

    Float get1D() {
        return std::generate_canonical<Float, std::numeric_limits<Float>::digits>(generator);
    }

private:
    std::default_random_engine generator;
};

template<typename T, int Bins>
class Histogram {
public:
    T data[Bins];
    Float weight;
    
    void splat(Float x, const T &v) {
        int i = int(round(x*Bins)) % Bins;
        data[i] += v;
        weight += 1;
    }
    
    int size() const { return Bins; }
    
    T get(int i) const {
        return data[i] * Float(Bins) / weight;
    }
};

struct Differential {
    Vector2f eu;
    Float dddu;
};

class Test {
public:
    guiding::Wrapper<guiding::BTree<1>> guiding;

    AggregateShape scene;
    Vector2f tx;
    Vector3c H;

    using Screen = Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic>;
    Screen screen;

    Vector2f screenPos, screenPxSize;
    
    RandomSampler sampler;
    
    Test() {
        guiding.settings.child.splitThreshold = 0.02f;
        guiding.settings.uniformProb = 0.1f;
        //guiding.settings.uniformProb = 1.f;

        if (0) {
            scene << new Rectangle(
                Vector2f(0.310, 0.395),
                Vector2f(1.110, 0.405)
            );

            scene << new Rectangle(
                Vector2f(1.110, -0.40),
                Vector2f(1.120, +0.40)
            );
        } else {
            //scene << new Circle(Vector2f(0.6, 0), 0.5);
            scene << new Circle(Vector2f(0.709, -0.25), 0.203);
            scene << new Circle(Vector2f(0.709, +0.25), 0.203);
        }

        tx << 0, 0;

        int scale = 30;
        screen.resize(16*scale, 12*scale);

        screenPos << -0.3, -0.6;
        screenPxSize << 1.6/screen.rows(), 1.2/screen.cols();
        
        H << 0, 0, 1;
    }

    Vector2f sampleMirror(Float u, Float &w, const Intersection &isect) {
        w = 1.f;
        return 2 * (isect.n.dot(isect.wi())) * isect.n - isect.wi();
    }

    Vector2f sampleDirection(Float u, Float &w, const Intersection &isect) {
#define USE_GOPO
#ifdef USE_GOPO
        if (isect.ray.depth > 0) {
            Vector2f result = sampleMirror(u, w, isect);
            w /= sqrt(isect.ray.k0()) / (2 * Pi);
            return result;
        }
#endif

        u *= 2 * Pi;
        w  = 2 * Pi; // pdf
        return Vector2f(cos(u), sin(u));
    }
    
    inline Vector3c connect(
        int depth,
        const Vector3c &J,
        const Vector2f &x,
        const Vector2f &y,
        const Vector2f &normal,
        Float k0,
        const Differential &diff
    ) {
        Vector2f wo = y - x;
        Float dist = wo.norm();
        wo /= dist;

#define INCORPORATE_VISIBILITY
#ifdef INCORPORATE_VISIBILITY
        //if (depth >= 2) {
        if (depth > 0) {
            if (wo.dot(normal) < 0)
                return Vector3c::Zero();
        }

        //if (depth >= 1) {
        {
            Ray shadowRay(x, wo);
            Intersection isect(shadowRay);
            isect.tMax = dist - Epsilon;
            scene.intersect(&isect);
            if (isect.valid())
                return Vector3c::Zero();
        }
#endif

        // see https://www3.nd.edu/~atassi/Teaching/AME%2060633/Notes/greens.pdf
        Complex green = sqrt(k0 / (8*Pi*dist)) * std::exp(Complex(0, -(k0 * dist - PiOver4)));
        Vector3c H = depth == 0 ?
            Vector3c(0, 0, -1) : // source
            J.cross(expand(wo)) * (
                Float(1) - Complex(0, 1) / max(k0 * dist, Float(1e-3)) /// @todo
            )
        ;

//#define DO_FILTERING
#ifdef DO_FILTERING
        Float K = 3.0f / k0;
        Float dpdu = K * k0 * (diff.dddu + diff.eu.dot(wo));
        if (abs(dpdu) > Epsilon) {
            green *= sin(dpdu) / dpdu;
        }
#endif

        return green * H;
    }
    
    inline void sample(Screen &screen) {
        Ray ray;
        ray.o = tx;
        ray.frequency = 30;
        ray.speed = 1;

        Vector3c J;
        Differential diff;
        Intersection isect(ray);

        Float contribution = 0;
        std::array<Float, 1> firstSample;
        Float firstPdf = 0;

        while (true) {
            Vector2f normal = isect.n;

            /// next event estimation
            if (ray.depth > 0) {
                auto H = connect(ray.depth, J, ray.o, tx, normal, ray.k0(), diff);
                contribution += std::abs(H.z());
            }

            if (ray.depth >= 0) {
                int Nnee = 128;//32 << (2*ray.depth);
                for (int i = 0; i < Nnee; ++i) {
                    float x = sampler.get1D() * screen.rows();
                    float y = sampler.get1D() * screen.cols();

                    if (false)
                    {
                        // disable pixel filtering
                        x = std::floor(x) + 0.5f;
                        y = std::floor(y) + 0.5f;
                    }

                    Vector2f rx;
                    rx.x() = screenPos.x() + x * screenPxSize.x();
                    rx.y() = screenPos.y() + y * screenPxSize.y();
                    
                    auto H = connect(ray.depth, J, ray.o, rx, normal, ray.k0(), diff);
                    screen(
                        std::min(long(x), screen.rows() - 1),
                        std::min(long(y), screen.cols() - 1)
                    ) += H.z() / Float(Nnee);
                }
            }

            /// random walk
            if (ray.depth >= 2)
                break;
            
            Float weight;

            {
                // sample direction
                std::array<Float, 1> sample = { sampler.get1D() };
                Float pdf = guiding.sample(sample);
                if (ray.depth == 0) {
                    firstSample = sample;
                    firstPdf = pdf;
                }
                ray.d = sampleDirection(sample[0], weight, isect);
                weight /= pdf;
            }

            isect.t = Infinity;
            scene.intersect(&isect);
            if (!isect.valid())
                break;

            Float cosTheta = isect.cosTheta();
            if (cosTheta < 1e-3)
                /// Grazing angles cause instabilities, since we would
                /// divide by a low cosine value. We ignore these outliers.
                break;
            
            auto H = connect(ray.depth, J, ray.o, isect.p, normal, ray.k0(), diff);
            J  = 2 * expand(isect.n).cross(H);
            J *= isect.t / cosTheta; // correction factor (1/(2*Pi) accounted for in pdf)
            J *= weight;
            
            ray.o = isect.p;

            diff.eu = Vector2f(-isect.n.y(), isect.n.x());
            diff.dddu = diff.eu.dot(isect.wi());

            ++ray.depth;
        }

        if (firstPdf > 0)
            guiding.splat(contribution * firstPdf, {}, 1/firstPdf, firstSample);
    }
    
    void run() {
        constexpr int THREAD_COUNT = 30;

        Screen screens[THREAD_COUNT];
        std::thread threads[THREAD_COUNT];
        volatile int samples[THREAD_COUNT];

        for (int tid = 0; tid < THREAD_COUNT; ++tid) {
            screens[tid].resizeLike(screen);
            samples[tid] = 0;
            threads[tid] = std::thread([this,&screens,&samples,tid] () {
                while (samples[tid] < 1000*1000 * 10) {
                    sample(screens[tid]);
                    ++samples[tid];
                }
            });
        }

        std::thread waitingThread = std::thread([&] () {
            using namespace std::chrono;

            auto start = high_resolution_clock::now();
            for (auto &thread : threads)
                thread.join();
            auto end = high_resolution_clock::now();

            Float elapsedTime = duration_cast<milliseconds>(end - start).count();
            cout << "rendering took " << elapsedTime << "ms" << endl;
        });

        Renderer renderer;
        while (true) {
            screen.setZero();
            for (int tid = 0; tid < THREAD_COUNT; ++tid) {
                screen += screens[tid] / samples[tid];
            }
            screen *= screen.size() / THREAD_COUNT;

            renderer.clear();
            render(renderer);
            renderer.poll();
        }
    }
    
    void render(Renderer &r) {
        /// draw screen
        for (int x = 0; x < screen.rows(); ++x) {
            for (int y = 0; y < screen.cols(); ++y) {
                Vector2f rx;
                rx.x() = screenPos.x() + x * screenPxSize.x();
                rx.y() = screenPos.y() + y * screenPxSize.y();

                //r.color(domainColoring(screen(x, y) / Float(2)));
                r.color(rdbu(
                    //real(screen(x, y)) / 8 + 0.5f
                    abs(screen(x, y)) / 16
                ));
                //r.color((RGB){ 255, 255, 255 });
                r.filledBox(rx, rx + screenPxSize);
            }
        }

        /// draw other stuff
        r.color((RGB){ 255, 0, 0 });
        r.point(tx);
        
        for (auto &a : scene.shapes) {
            if (auto rect = dynamic_cast<Rectangle *>(a.get())) {
                r.box(rect->min(), rect->max());
            }

            if (auto circle = dynamic_cast<Circle *>(a.get())) {
                r.circle(circle->center(), circle->radius());
            }
        }

        /// draw distribution
        constexpr int steps = 1024;
        std::array<Float, 1> sample;

        Vector2f lastPoint;

        for (int i = 0; i <= steps; ++i) {
            sample[0] = ((i % steps) + 0.5f) / steps;
            Float pdf = guiding.pdf(sample);
            Vector2f point = tx + 0.05f * pdf * Vector2f(
                std::cos(sample[0] * 2 * Pi),
                std::sin(sample[0] * 2 * Pi)
            );

            if (i) {
                //r.color((RGB){ 255, 255, 255 });
                //r.line(tx, lastPoint);
                r.color((RGB){ 0, 0, 0 });
                r.line(lastPoint, point);
            }

            lastPoint = point;
        }
    }
};

int main(int argc, const char **argv) {
    Test test;
    test.run();
    return 0;
}
