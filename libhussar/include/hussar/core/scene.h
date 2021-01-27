#ifndef HUSSAR_CORE_SCENE_H
#define HUSSAR_CORE_SCENE_H

#include <radar/radar.h>
#include <hussar/hussar.h>
#include <hussar/core/emitter.h> /// @todo hack!

namespace hussar {

class Scene {
public:
    radar::RFConfig rfConfig;
    
    NFAntenna rx;
    NFAntenna tx;
};

}

#endif
