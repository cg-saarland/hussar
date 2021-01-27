#ifndef VISUALIZER_PLOTS_H
#define VISUALIZER_PLOTS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <hussar/hussar.h>
#include <hussar/core/geometry.h>
#include <hussar/core/image.h>
#include <radar/radar.h>

#include <gl.h>

float hue2rgb(float p, float q, float t) {
    if(t < 0) t += 1;
    if(t > 1) t -= 1;
    if(t < 1.f/6) return p + (q - p) * 6 * t;
    if(t < 1.f/2) return q;
    if(t < 2.f/3) return p + (q - p) * (2.f/3 - t) * 6;
    return p;
}

hussar::Vector4f hsl(float h, float s, float l) {
    float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    
    return hussar::Vector4f(
        hue2rgb(p, q, h + 1.f/3),
        hue2rgb(p, q, h),
        hue2rgb(p, q, h - 1.f/3),
        1.f
    );
}

hussar::Vector4f falseColorComplex(hussar::Complex v) {
    return hsl(std::arg(v) / (2 * M_PI) + 0.5f, 1.f, std::abs(v));
}

hussar::Vector4f falseColor(float v) {
    hussar::Vector4f c = hussar::Vector4f::Ones();
    v = std::max(0.f, std::min(1.f, v));
    
    if (v < 0.25) {
        c.x() = 0.0;
        c.y() = 4.0 * v;
    } else if (v < 0.5) {
        c.x() = 0.0;
        c.z() = 1.0 + 4.0 * (0.25 - v);
    } else if (v < 0.75) {
        c.x() = 4.0 * (v - 0.5);
        c.z() = 0.0;
    } else {
        c.y() = 1.0 + 4.0 * (0.75 - v);
        c.z() = 0.0;
    }
    return c;
}

struct Plot {
    GLuint gl_tex;
    hussar::Image<hussar::Vector4f> image;
    int scale = 1;
    
    Plot() : image(256, 256) {
        glGenTextures(1, &gl_tex);
        glBindTexture(GL_TEXTURE_2D, gl_tex);

        // Setup filtering parameters for display
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        // Upload pixels into texture
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    }

    void rect(float x, float y, const hussar::Vector4f &color, int size = 5, int borderSize = 1) {
        int xi = std::floor(x * image.width());
        int yi = std::floor(y * image.width());

        size = (size - 1) / 2;
        for (int xo = -size; xo <= +size; ++xo)
            for (int yo = -size; yo <= +size; ++yo) {
                int xx = xi+xo;
                int yy = yi+yo;

                int border = std::max(std::abs(xo), std::abs(yo)) > (size - borderSize);

                if (image.inBounds(xx, yy))
                    image.at(xx, yy) = border ?
                        hussar::Vector4f(1.f, 1.f, 1.f, 1.f) :
                        color;
            }
    }

    void draw() {
        glBindTexture(GL_TEXTURE_2D, gl_tex);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA,
            image.width(), image.height(),
            0, GL_RGBA,
            GL_FLOAT,
            image.data()
        );
        
        ImGui::Image((void *)(intptr_t)gl_tex, ImVec2(
            image.width() * scale,
            image.height() * scale
        ));
    }

    void fill(std::function<hussar::Vector4f (float x, float y)> fn) {
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x) {
                image.at(x, y) = fn(
                    (x + 0.5f) / image.width(),
                    (y + 0.5f) / image.height()
                );
            }
    }
    
    void draw(std::function<hussar::Vector4f (float x, float y)> fn) {
        fill(fn);
        draw();
    }
};

struct RangeDopplerPlot {
    Plot plot;

    void draw(const hussar::RadarFrame &frame, const hussar::RadarFrame::PIndex &argmax) {
        ImGui::SliderInt("Scale", &plot.scale, 1, 8, "%dx");

        //float ax = (radar::modulo_one(argmax.chirp / frame.config().chirpCount + 0.5) - 0.5) * 3 + 0.5;
        float ax = (radar::modulo_one(argmax.channel / frame.config().channelCount + 0.5) - 0.5) * 1 + 0.5;
        float ay = argmax.sample / frame.config().samplesPerChirp * 3;
        
        plot.draw([&](float x, float y) {
            float d = std::hypot(x - ax, y - ay);
            if (d < 0.02) {
                float c = d < 0.015;
                return hussar::Vector4f(c, c, c, 1.f);
            }

            hussar::RadarFrame::PIndex index;
            index.sample = y / 3 * frame.config().samplesPerChirp;
            //index.chirp = radar::modulo_one((x - 0.5f) / 3) * frame.config().chirpCount;
            index.channel = radar::modulo_one(x - 0.5f) * frame.config().channelCount;
            return falseColor(1e-1 * std::abs(frame(index)));
        });
    }
};

hussar::RadarFrame::Index findArgmaxWithRef(const hussar::RadarFrame &a, const hussar::RadarFrame &ref) {
    size_t i = 0;
    float best = std::abs(a(i)) - std::abs(ref(i));
    for (size_t s = 1; s < a.sampleCount(); ++s) {
        float c = std::abs(a(s)) - std::abs(ref(s));
        if (c > best) {
            i = s;
            best = c;
        }
    }
    
    return a.makeIndex(i);
}

#pragma GCC diagnostic pop

#endif
