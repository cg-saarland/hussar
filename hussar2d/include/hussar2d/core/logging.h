#ifndef HUSSAR2D_CORE_LOGGING_H
#define HUSSAR2D_CORE_LOGGING_H

#include <hussar2d/hussar2d.h>
#include <stdarg.h>
#include <stdio.h>
#include <cassert>

namespace hussar2d {

enum ELogLevel {
    ETrace = 0,
    EDebug = 100,
    EInfo  = 200,
    EWarn  = 300,
    EError = 400
};

inline void Log(ELogLevel level, const char *fmt, ...) __attribute__((format (printf, 2, 3)));
inline void Log(ELogLevel level, const char *fmt, ...) {
    switch (level) {
    case ETrace: printf("[Trace]: "); break;
    case EDebug: printf("[Debug]: "); break;
    case EInfo:  printf("[Info ]: "); break;
    case EWarn:  printf("[Warn ]: "); break;
    case EError: printf("[Error]: "); break;
    }
    
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    assert(level < EError); /// @todo this should throw an exception
}

#ifdef NDEBUG
#define Assert(cond, msg) ((void) 0)
#else
#define Assert(cond, msg) do { \
    if (!(cond)) \
        Log(EError, "%s in %s:%i", (msg), __FILE__, __LINE__); \
} while (0)
#endif

}

#endif
