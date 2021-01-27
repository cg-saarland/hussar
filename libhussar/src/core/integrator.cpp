#include <hussar/core/integrator.h>
#include <hussar/io/exr.h>

namespace hussar {

template<> void Channelizer::transfer(ExrFile::ChannelName channel, Integrator::DebugElement &el) {
    transfer(channel("contribution"), el.contribution);
    transfer(channel("dphase.L"),     el.dphase);
    transfer(channel("distance.L"),   el.distance);
    transfer(channel("weight.L"),     el.weight);
}

}
