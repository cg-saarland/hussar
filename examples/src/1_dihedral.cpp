#include <iostream>
#include <fstream>

#include <radar/units.h>

#include <hussar/hussar.h>
#include <hussar/core/frame.h>
#include <hussar/core/mesh.h>
#include <hussar/core/geometry.h>
#include <hussar/core/scene.h>
#include <hussar/core/emitter.h>
#include <hussar/integrators/path.h>

#include <hussar/arch/cpu.h>
#include <hussar/arch/gpu.h>

#include "utils.h"

using namespace std;
using namespace hussar;
using namespace radar;

int main() {
    /// MARK: Radar configuration

    // configure the parameters of the FMCW ramps
    radar::RFConfig rf;
    rf.antennaDelay = 0.43_ns;
    
    rf.startFreq = 77_GHz;
    rf.adcRate   = 10_MHz;
    rf.freqSlope = 29.982_MHz / 1_us;
    rf.idleTime  = 100_us;
    rf.rampTime  = 60_us;
    
    rf.freqSlope = 60_MHz / 1_us;
    rf.adcRate   = 5_MHz;

    // configure the parameters of captured frames
    radar::FrameConfig frameConfig;
    frameConfig.chirpCount      = 128;
    frameConfig.samplesPerChirp = 256;
    frameConfig.channelCount    = 4;

    // higher sample counts produce results will less noise, but will
    // take longer to compute
    long sampleCount = 200*1000;

    /// MARK: Scene mesh

    TriangleMesh mesh;

    /// edge length of the two rectangles that make up the dihedral reflector
    float size = 50_mm;

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

    /// MARK: Simulation

    RadarFrame frame;

    // construct our scene description
    auto scene = hussar::make_shared<Scene>();

    // use the FMCW ramp configuration we have created earlier
    scene->rfConfig = rf;

    // construct our path tracing integrator
    auto integrator = hussar::make_shared<PathTracer>();

    // debug images provide some insights into surface currents
    integrator->produceDebugImage = true;
    integrator->configureFrame(frameConfig);

    // for this example, we will use the CPU backend
    // note that running on GPU is as simple as replacing 'cpu::Backend' with 'gpu::Backend'
    cpu::Backend backend { mesh, *integrator };

    // all simulated frames will end up concatenated in a single file
    std::ofstream file("dihedral.SIM");

    // simulate the Radar response for a range of angles
    for (float angleDeg = -55; angleDeg <= +55; angleDeg += 0.25) {
        // we are rotating our antennas around the y axis
        Matrix33f rotation = Eigen::AngleAxisf((angleDeg-45) / 180 * Pi, Vector3f::UnitY()).toRotationMatrix();

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

        // run the simulation
        integrator->run(backend, *scene, sampleCount);

        // every ten steps output a debug image and report the current angle
        static int i = 0;
        if (++i >= 10) {
            std::cout << "angle: " << angleDeg << std::endl;
            integrator->saveDebugImage("dihedral");
            i = 0;
        }

        frame = integrator->fetchFrame();
        writeFrameToFile(file, frame);
    }
}
