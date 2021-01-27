#ifndef RCS_UTILS_H
#define RCS_UTILS_H

#include <iostream>
#include <chrono>
#include <fstream>

#include <hussar/hussar.h>

class Timer {
public:
    Timer(const std::string &msg) : message(msg) {
        std::cout << "Starting " << message << "..." << std::endl;
    }

    ~Timer() {
        auto endTime = std::chrono::steady_clock::now();
        
        typedef std::chrono::microseconds us;
        float time = std::chrono::duration_cast<us>(endTime - startTime).count() / 1000.f;
        std::cout << "- took " << time << " ms" << std::endl;
    }

private:
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::string message;
};

void writeFrameToFile(std::ofstream &file, const hussar::RadarFrame &frame) {
    for (size_t i = 0; i < frame.sampleCount(); ++i) {
        auto v = frame(i);
        file.write((const char *)(&v), 2 * sizeof(radar::Float));
    }
    file.flush();
}

void saveFrame(const hussar::RadarFrame &frame, const std::string &path) {
    std::ofstream file(path + ".SIM");
    writeFrameToFile(file, frame);
}

#endif
