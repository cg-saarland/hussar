#include <exception>

#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

#include <hussar/io/exr.h>

namespace hussar {

// Use class's direction transfer function
template<>
void Channelizer::transfer(ChannelName channel, float& el) {
    directedTransfer(channel, el);
}

template<>
void Channelizer::transfer(Channelizer::ChannelName channel, radar::complex<float>& el) {
    float &firstReal = el.real();
    transfer(channel("Re"), firstReal);

    float &firstImag = el.imag();
    transfer(channel("Im"), firstImag);
}

}
