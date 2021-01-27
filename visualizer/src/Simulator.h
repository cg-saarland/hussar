#include <hussar/hussar.h>

#include <hussar/core/frame.h>
#include <hussar/core/geometry.h>
#include <hussar/core/scene.h>
#include <hussar/core/emitter.h>
#include <hussar/integrators/path.h>

#include <hussar/arch/cpu.h>

#include "Plots.h"

#include <gl.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

constexpr bool DO_DIHEDRAL_SCENE = true;

class Simulator {
public:
    using Integrator = hussar::PathTracer;
    using Backend = hussar::cpu::Backend;

    hussar::TriangleMesh mesh;
    hussar::ref<Backend> backend;
    Integrator::DebugImage debugImage;

    bool interruptIntegrator;
    hussar::ref<hussar::Scene> scene;
    hussar::ref<Integrator> integrator;
    hussar::RadarFrame simulation;

    std::thread renderThread;
    float lastAngle = std::numeric_limits<float>::infinity();
    
    struct DebugTexture {
        GLuint gl_tex;
        hussar::Image<hussar::Vector4f> image;
        
        DebugTexture() : image(128, 64) {
            glGenTextures(1, &gl_tex);
            glBindTexture(GL_TEXTURE_2D, gl_tex);

            // Setup filtering parameters for display
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

            // Upload pixels into texture
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        }
        
        void draw(
            Integrator::DebugImage &debug,
            std::function<hussar::Vector4f (Integrator::DebugImage::ElementType &)> fn
        ) {
            using namespace hussar;

            // show an inset from the center of the debug image
            int xOffset = debug.width() / 2 - image.width() / 2;
            int yOffset = debug.height() / 2 - image.height() / 2;
            
            for (int y = 0; y < image.height(); ++y)
                for (int x = 0; x < image.width(); ++x) {
                    auto &dbg = debug.at(x + xOffset, y + yOffset);
                    image.at(x, y) = dbg.invPdfs < 1e-3 ?
                        Vector4f(1.f, 1.f, 1.f, ((x%8)<4) ^ ((y%8)<4) ? 0.05f : 0.0f) : // checkerboard pattern
                        fn(dbg)
                    ;
                }
            
            glBindTexture(GL_TEXTURE_2D, gl_tex);
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGBA,
                image.width(), image.height(),
                0, GL_RGBA,
                GL_FLOAT,
                image.data()
            );
            
            ImGui::Image((void *)(intptr_t)gl_tex, ImVec2(256, 128));
        }
    };
    
    std::vector<DebugTexture> debugTextures;
    
    void initTextures() {
        debugTextures.resize(3);
    }
    
    void initialize(const radar::RFConfig &rf) {
        using namespace hussar;

        scene = make_shared<Scene>();
        scene->rfConfig = rf;

        radar::FrameConfig frameConfig;
        frameConfig.chirpCount      = 128;
        frameConfig.samplesPerChirp = 256;
        frameConfig.channelCount    = 4;

        integrator = make_shared<Integrator>();
        integrator->configureFrame(frameConfig);
        integrator->produceDebugImage = true;
        
        /// edge length of the two rectangles that make up the dihedral reflector
        float size = 100_mm;

        // first side of the dihedral reflector
        mesh.addBox(
            Vector3f(-2_mm, +0_mm, +0_mm),
            Vector3f(-0_mm, +size, +size)
        );

        // second side of the dihedral reflector
        mesh.addBox(
            Vector3f(+0_mm, +0_mm, -2_mm),
            Vector3f(+size, +size, -0_mm)
        );

        backend = make_shared<Backend>(mesh, *integrator);
        
        //
        
        initTextures();
    }

    hussar::RadarFrame *simulate(float angle) {
        using namespace hussar;

        if constexpr(DO_DIHEDRAL_SCENE) {
            angle -= PiOver4;
        } else {
            angle *= 2;
        }

        if (angle != lastAngle) {
            // need to restart integrator
            
            if (renderThread.joinable()) {
                interruptIntegrator = true;
                renderThread.join();
            }
            
            // we are rotating our antennas around the y axis
            Matrix33f rotation = Eigen::AngleAxisf(angle, Vector3f::UnitY()).toRotationMatrix();

            // the local coordinate system of our antennas
            // the z-vector (3rd column) is the direction we are looking at
            Matrix33f facing;
            facing <<
                0, 0, -1,
                0, -1, 0,
                -1, 0, 0;

            // place the antennas
            scene->rx = NFAntenna {
                rotation * Vector3f(896_mm, 67_mm, -5_mm), // location of the receive antenna
                rotation * facing,                         // local coordinate system of the antenna
                AWRAngularDistribution()                   // radiation pattern
            };
            scene->tx = NFAntenna {
                rotation * Vector3f(896_mm, 67_mm, -7_mm), // location of the transmit antenna
                rotation * facing,                         // local coordinate system of the antenna
                AWRAngularDistribution()                   // radiation pattern
            };

            lastAngle = angle;

            // MARK: - run integrator

            renderThread = std::thread([&] {
                interruptIntegrator = false;
                integrator->run(*backend, *scene, 16*1024*1024, &interruptIntegrator);
                //integrator->saveDebugImage("debug");
            });
        }

        simulation  = integrator->fetchFrame();
        simulation *= 1e-2;
        simulation(0) += 1e-3;
        return &simulation;
    }
    
    void render() {
        using namespace hussar;

        debugImage = integrator->getDebugImage();

        double totalWeight = 0;
        debugImage.each([&](auto &e) { totalWeight += e.weight; });

        // update textures
        ImGui::Begin("Simulator");
        debugTextures[0].draw(debugImage, [](auto &e) {
            return falseColor(e.distance / 8);
        });
        debugTextures[1].draw(debugImage, [](auto &e) {
            return falseColorComplex(e.contribution / Float(50));
        });
        debugTextures[2].draw(debugImage, [&](auto &e) {
            return falseColor(3e+2 * e.weight / totalWeight);
        });
        ImGui::End();
    }
};

#pragma GCC diagnostic pop
