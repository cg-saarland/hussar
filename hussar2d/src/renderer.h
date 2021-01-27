#include <hussar2d/hussar2d.h>
#include "SDL.h"

namespace hussar2d {

void DrawCircle(SDL_Renderer * renderer, int32_t centreX, int32_t centreY, int32_t radius) {
   const int32_t diameter = (radius * 2);

   int32_t x = (radius - 1);
   int32_t y = 0;
   int32_t tx = 1;
   int32_t ty = 1;
   int32_t error = (tx - diameter);

   while (x >= y)
   {
      //  Each of the following renders an octant of the circle
      SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
      SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
      SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
      SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
      SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
      SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
      SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
      SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

      if (error <= 0)
      {
         ++y;
         error += ty;
         ty += 2;
      }

      if (error > 0)
      {
         --x;
         tx += 2;
         error += (tx - diameter);
      }
   }
}

struct HSL {
    Float h, s, l;
};

struct RGB {
    uint8_t r, g, b;
};

float hue2rgb(float v1, float v2, float vH) {
    if (vH < 0) vH += 1;
    if (vH > 1) vH -= 1;
    
    if ((6 * vH) < 1) return (v1 + (v2 - v1) * 6 * vH);
    if ((2 * vH) < 1) return v2;
    if ((3 * vH) < 2) return (v1 + (v2 - v1) * ((2.0f / 3) - vH) * 6);

    return v1;
}

RGB hsl2rgb(HSL hsl) {
    uint8_t r, g, b;
    r = g = b = 0;

    if (hsl.s == 0) {
        r = g = b = hsl.l * 255;
    } else {
        float v1, v2;
        float hue = (float)hsl.h / (2 * Pi);

        v2 = (hsl.l < 0.5) ? (hsl.l * (1 + hsl.s)) : ((hsl.l + hsl.s) - (hsl.l * hsl.s));
        v1 = 2 * hsl.l - v2;

        r = 255 * hue2rgb(v1, v2, hue + (1.0f / 3));
        g = 255 * hue2rgb(v1, v2, hue);
        b = 255 * hue2rgb(v1, v2, hue - (1.0f / 3));
    }

    return { r, g, b };
}

RGB domainColoring(Complex v, Float a = 0.5) {
    return hsl2rgb({
        arg(v),
        1,
        1 - pow(a, abs(v)),
    });
}

struct GradientStop {
    float r, g, b;
};

RGB rdbu(Float v) {
    static const GradientStop stops[] = {
        { 0.40392156862745099,  0.0                ,  0.12156862745098039},
        { 0.69803921568627447,  0.09411764705882353,  0.16862745098039217},
        { 0.83921568627450982,  0.37647058823529411,  0.30196078431372547},
        { 0.95686274509803926,  0.6470588235294118 ,  0.50980392156862742},
        { 0.99215686274509807,  0.85882352941176465,  0.7803921568627451 },
        { 0.96862745098039216,  0.96862745098039216,  0.96862745098039216},
        { 0.81960784313725488,  0.89803921568627454,  0.94117647058823528},
        { 0.5725490196078431 ,  0.77254901960784317,  0.87058823529411766},
        { 0.2627450980392157 ,  0.57647058823529407,  0.76470588235294112},
        { 0.12941176470588237,  0.4                ,  0.67450980392156867},
        { 0.0196078431372549 ,  0.18823529411764706,  0.38039215686274508},
    };

    const int N = (sizeof(stops) / sizeof(GradientStop)) - 1;

    float i = v * N;
    int ii = floor(i);
    float io = i - ii;

    if (ii <  0) { ii = 0; io = 0; }
    if (ii >= N) { ii = N; io = 0; }

    return (RGB){
        uint8_t( 255 * (stops[ii].r*(1-io) + io*stops[ii+1].r) ),
        uint8_t( 255 * (stops[ii].g*(1-io) + io*stops[ii+1].g) ),
        uint8_t( 255 * (stops[ii].b*(1-io) + io*stops[ii+1].b) ),
    };
}

class Renderer {
private:
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    
    Vector2f size = Vector2f(1200, 900);
    Vector2f bounds[2];
    Float scale;
    
    Vector2f local(const Vector2f &global) const {
        Vector2f l = size.array() * (global - bounds[0]).array() / (bounds[1] - bounds[0]).array();
        return l;
    }
    
public:
    Renderer() {
        Float margin = 0.4;
        Float aspect = size.x() / size.y();
        
        bounds[0] << (0-margin), (-0.5-margin)/aspect;
        bounds[1] << (1+margin), (+0.5+margin)/aspect;

        scale = size.x() / (bounds[1] - bounds[0]).x();
        
        if (SDL_Init(SDL_INIT_VIDEO))
            exit(1);
        if (SDL_CreateWindowAndRenderer(size.x(), size.y(), 0, &window, &renderer))
            exit(1);
    }
    
    void color(const RGB &rgb) {
        SDL_SetRenderDrawColor(renderer, rgb.r, rgb.g, rgb.b, SDL_ALPHA_OPAQUE);
    }
    
    void point(const Vector2f &a) {
        auto al = local(a);
        DrawCircle(renderer, al.x(), al.y(), 3);
    }
    
    void line(const Vector2f &a, const Vector2f &b) {
        auto al = local(a);
        auto bl = local(b);
        SDL_RenderDrawLine(renderer, al.x(), al.y(), bl.x(), bl.y());
    }
    
    void box(const Vector2f &a, const Vector2f &b) {
        auto al = local(a);
        auto bl = local(b);
        SDL_RenderDrawLine(renderer, al.x(), al.y(), bl.x(), al.y());
        SDL_RenderDrawLine(renderer, bl.x(), al.y(), bl.x(), bl.y());
        SDL_RenderDrawLine(renderer, bl.x(), bl.y(), al.x(), bl.y());
        SDL_RenderDrawLine(renderer, al.x(), bl.y(), al.x(), al.y());
    }
    
    void circle(const Vector2f &a, Float radius) {
        auto al = local(a);
        DrawCircle(renderer, al.x(), al.y(), radius * scale);
    }

    void filledBox(const Vector2f &a, const Vector2f &b) {
        auto al = local(a);
        auto bl = local(b);
        
        SDL_Rect rect;
        rect.x = al.x();
        rect.y = al.y();
        rect.w = bl.x() - rect.x + 1;
        rect.h = bl.y() - rect.y + 1;
        SDL_RenderFillRect(renderer, &rect);
    }

    void clear() {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    }
    
    void poll() {
        SDL_Event event;
        SDL_RenderPresent(renderer);
        while (SDL_WaitEventTimeout(&event, 100)) {
            if (event.type == SDL_QUIT) {
                if (renderer) {
                    SDL_DestroyRenderer(renderer);
                }
                if (window) {
                    SDL_DestroyWindow(window);
                }
                
                SDL_Quit();
                exit(0);
            }
        }
    }
};

}
