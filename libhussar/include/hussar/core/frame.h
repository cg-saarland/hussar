#ifndef HUSSAR_CORE_FRAME_H
#define HUSSAR_CORE_FRAME_H

#include <radar/radar.h>
#include <hussar/hussar.h>
#include <hussar/core/allocator.h>

namespace hussar {

using RadarFrame = radar::Frame<Allocator<radar::Complex>>;

}

#endif
