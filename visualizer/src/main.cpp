#include <awrcapture/awrcapture.h>
#include <radar/radar.h>
#include <radar/units.h>
#include <imgui.h>

#include "StepperLib.h"
#include "MaterialMeasurement.h"
#include "Simulator.h"
#include "Plots.h"

#include <vector>
#include <fstream>
#include <filesystem>
#include <mutex>

namespace fs = std::filesystem;

ImVec4 clear_color = ImVec4(0.16, 0.16, 0.18, 1.0);

ImFont *AddDefaultFont(float pixel_size) {
    ImFontConfig config;
    config.SizePixels = pixel_size;
    config.OversampleH = config.OversampleV = 1;
    config.PixelSnapH = true;

    ImGuiIO &io = ImGui::GetIO();
    ImFont *font = io.Fonts->AddFontDefault(&config);

    return font;
}

const float RADIANS_PER_STEP = 2*M_PI / 2037.8864 * 1.025f;

radar::RFConfig rf;
awrcapture::ControlPort controlPort;
awrcapture::DataPort dataPort;
Simulator simulator;
MaterialMeasurement matm;
std::thread updateThread;
std::mutex updateMutex;
hussar::RadarFrame *frame = nullptr;
int frameCounter = 0;
bool connected = false;

hussar::RadarFrame ref;
bool refBuilt = false;
bool refActive = false;
int refCount = 0;

void update() {
    std::unique_lock lock(updateMutex);

    matm.slib.poll();
    dataPort.poll([](radar::Frame<> &f) {
        if (refActive) {
            ref += f;
            if (++refCount >= 50) {
                std::cout << "reference built!" << std::endl;
                ref *= 1.f / refCount;
                refActive = false;
                refBuilt = true;
            }
        }
        
        frame = &f;
        ++frameCounter;
    });

    if (!connected)
        frame = simulator.simulate(matm.slib.steppers[0].target * RADIANS_PER_STEP);

    matm.update(frame);
}

ImFont *largeFont;
void initialize() {
    AddDefaultFont(13);
    largeFont = AddDefaultFont(100);
    
    rf.antennaDelay = 0.43_ns;
    
    rf.startFreq = 77_GHz;
    rf.adcRate   = 10_MHz;
    rf.freqSlope = 29.982_MHz / 1_us;
    rf.idleTime  = 100_us;
    rf.rampTime  = 60_us;
    
    rf.freqSlope = 60_MHz / 1_us;
    rf.adcRate   = 5_MHz;

    radar::FrameConfig frameConfig;
    frameConfig.samplesPerChirp = 256;
    frameConfig.chirpCount = 128;
    frameConfig.channelCount = 4;
    
    dataPort.configureFrame(frameConfig);

    //
    
    simulator.initialize(rf);
    //matm.initialize();

    updateThread = std::thread([]() {
        while (true) {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    });
}

void expavg(float &v, float x, float step) {
    v = (1-step) * v + step * x;
}

enum HistogramMode {
    HIST_ABS = 0,
    HIST_REAL = 1,
    HIST_IMAG = 2
};

const char *to_string(HistogramMode hm) {
    switch (hm) {
    case HIST_ABS : return "|.|";
    case HIST_REAL: return "Re";
    case HIST_IMAG: return "Im";
    default: return "?";
    }
}

void render() {
    std::unique_lock lock(updateMutex);

    static const char *stTargetNames[] = { "Sample (t)", "Reflector (t)" };
    static const char *stStateNames[] = { "Sample", "Reflector" };
    
    static std::string fpgaVersion = "";
    static hussar::RadarFrame::Index max;
    
    static float distance = 0;
    static float velocity = 0;
    
    static std::vector<float> histogram;
    static float minV = -20.00f;
    static float maxV = +40.00f;
    static bool histogramUseLog = true;
    static HistogramMode histogramMode = HIST_ABS;
    static hussar::RadarFrame::PIndex argmax;
    
    static int chirp = 0;
    static int channel = 0;
    
    StepperLib<2> &slib = matm.slib;

    static RangeDopplerPlot rdplot;
    
    static bool showDemoWindow = false;
    if (showDemoWindow)
        ImGui::ShowDemoWindow(&showDemoWindow);
    
    if (!connected)
        simulator.render();

    //if (frame) {
    //    ImGui::Begin("Plots");
    //    rdplot.draw(*frame, argmax);
    //    ImGui::End();
    //}
    
    {
        ImGui::Begin("AWR");
        
        if (!connected && ImGui::Button("Connect")) {
            fpgaVersion = controlPort.readFPGAVersion().str();
            
            controlPort.systemConnect();
            controlPort.configFPGAGen(awrcapture::DeviceFamily::AWR1243);
            controlPort.configPacketData();
            controlPort.recordStart();
            
            connected = true;
        }
        
        //if (ImGui::Button("Start measurement"))
        //    vMeasure.start(distance);
        
        if (frame) {
            if (ImGui::Button("Start reference")) {
                ref.configure(frame->config());
                
                refActive = true;
                refCount = 0;
            }
            
            ImGui::SameLine();
            matm.render();
            
            histogram.resize(frame->config().samplesPerChirp);
            for (size_t i = 0; i < histogram.size(); ++i) {
                histogram[i] = 0.f;
                
                hussar::RadarFrame::Index idx;
                idx.sample = i;
                idx.channel = channel;
                idx.chirp = chirp;
                
                switch (histogramMode) {
                case HIST_ABS : histogram[i] = std::abs ((*frame)(idx)); break;
                case HIST_REAL: histogram[i] = std::real((*frame)(idx)); break;
                case HIST_IMAG: histogram[i] = std::imag((*frame)(idx)); break;
                }
                
                if (refBuilt)
                    histogram[i] -= std::abs(ref(idx));

                if (histogramUseLog) {
                    histogram[i] = 20 * std::log(
                        std::max(histogram[i], float(1e-5))
                    ) / std::log(10);
                }
            }
            
            //if (frameCounter % 8 == 0) {
                max = refBuilt ? findArgmaxWithRef(*frame, ref) : frame->argmax();
                
                argmax = frame->frequencyEstimation(max);
                expavg(distance, argmax.distance(rf, frame->config()) / 2.f, 0.05);
                expavg(velocity, argmax.velocity(rf, frame->config()), 0.05);
            //}
            
            auto pval = (*frame)(argmax);
            
            ImGui::Text("fpga:   %s", fpgaVersion.c_str());
            ImGui::Text("max:    chan=%.1f, samp=%.1f, chir=%.1f", argmax.channel, argmax.sample, argmax.chirp);
            //ImGui::Text("value:  %.3f + %.3fi", pval.real(), pval.imag());
            ImGui::Text("value:  %.1f", std::abs(pval));
            ImGui::Text("frames: %d", frameCounter);
            
            ImGui::PushFont(largeFont);
            ImGui::Text("%.2fm @ %.2fm/s", distance, velocity);
            ImGui::PopFont();
            
            ImGui::PushItemWidth(240);
            
            ImGui::Checkbox("FFT", &dataPort.performFFT);
            ImGui::SameLine();
            ImGui::Checkbox("Log", &histogramUseLog);
            ImGui::SameLine();
            if (ImGui::BeginCombo("##hmode", to_string(histogramMode))) {
                for (int i = 0; i < 3; ++i) {
                    auto v = HistogramMode(i);
                    bool isSelected = histogramMode == v;
                    
                    if (ImGui::Selectable(to_string(v), isSelected))
                        histogramMode = v;
                    if (isSelected)
                        ImGui::SetItemDefaultFocus();
                }

                ImGui::EndCombo();
            }

            float maxScale = histogramUseLog ? 60 : 5;
            ImGui::SliderFloat("##minV", &minV,  -maxScale, 0.0f, histogramUseLog ? "min %.2f dB" : "min %.2f");
            ImGui::SameLine();
            ImGui::SliderFloat("##maxV", &maxV,  0.0f, maxScale, histogramUseLog ? "max %.2f dB" : "max %.2f");
            
            ImGui::SliderInt("##chirp", &chirp, 0, frame->config().chirpCount-1, "chirp #%d");
            ImGui::SameLine();
            ImGui::SliderInt("##chan.", &channel, 0, frame->config().channelCount-1, "channel #%d");
            
            ImGui::PopItemWidth();
            
            //vMeasure.record(velocity, distance);
        }
        
        ImGui::PlotLines(
            "##Histogram",
            (float *)histogram.data(), int(histogram.size()),
            0, NULL, minV, maxV, ImVec2(ImGui::GetContentRegionAvailWidth(), 300)
        );
        
        ImGui::End();
    }
    
    {
        ImGui::Begin("Stepper motors");
        
        /*if (slib.connected()) {*/
            for (int i = 0; i < slib.STEPPER_COUNT; ++i) {
                if (ImGui::SliderInt(stTargetNames[i], &slib.steppers[i].target, -300, +300))
                    slib.steppers[i].sendTarget();
                ImGui::SliderInt(stStateNames[i], &slib.steppers[i].state, -300, +300);
            }
            
            if (ImGui::Button("Calibrate")) {
                for (int i = 0; i < slib.STEPPER_COUNT; ++i)
                    slib.steppers[i].calibrate();
            }
        /*} else*/ {
            if (ImGui::Button("Connect Steppers")) {
                std::string path = "/dev";
                for (const auto &entry : fs::directory_iterator(path)) {
                    auto fname = entry.path().filename().string();
                    if (fname.rfind("cu.usbmodem1", 0) == 0 || fname.rfind("ttyACM", 0) == 0) {
                        path = entry.path().string();
                        break;
                    }
                }
                
                slib.connect(path);
            }
        }
        
        ImGui::End();
    }
}
