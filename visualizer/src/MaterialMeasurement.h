#ifndef MaterialMeasurement_h
#define MaterialMeasurement_h

#include <fstream>
#include <hussar/core/frame.h>

class MaterialMeasurement {
public:
    StepperLib<2> slib;
    
    void render() {
        if (!active) {
            if (ImGui::Button("Material measurement"))
                start();
            return;
        }
        
        ImGui::Text("measuring...");
    }

    void update(hussar::RadarFrame *frame) {
        if (active)
            tick(frame);
    }
    
private:
    bool active = false;
    int timer = 0;
    std::ofstream file;
    
    hussar::RadarFrame acc;
    
    void target(int t) {
        slib.steppers[0].target = t;
        slib.steppers[0].sendTarget();
    }
    
    void tick(hussar::RadarFrame *frame) {
        constexpr int FRAME_COUNT = 1;
        
        if (--timer > 0)
            return;
        
        if (-timer == 0) {
            acc.configure(frame->config());
        }
        
        if (-timer < FRAME_COUNT) {
            acc += *frame;
            return;
        }
        
        acc *= 1.f / FRAME_COUNT;
        
        /// update stepper
        
        if (slib.steppers[0].target > 300) {
            stop();
            return;
        }
        
        slib.steppers[0].target += 1;
        slib.steppers[0].sendTarget();
        timer = 15;
        
        /// log data in the meanwhile
        
        for (size_t i = 0; i < acc.sampleCount(); ++i) {
            auto v = acc(i);
            file.write((const char *)(&v), 2 * sizeof(float));
        }
    }
    
    void start() {
        active = true;
        target(-300);
        timer = 15;
        
        file = std::ofstream("concave-sim.txt");
    }
    
    void stop() {
        active = false;
        file.close();
    }
};

#endif
