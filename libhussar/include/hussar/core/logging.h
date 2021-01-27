#ifndef HUSSAR_CORE_LOGGING_H
#define HUSSAR_CORE_LOGGING_H

#include <hussar/hussar.h>

#include <stdarg.h>
#include <stdio.h>
#include <cassert>

namespace hussar {

enum ELogLevel {
    ETrace = 0,
    EDebug = 100,
    EInfo  = 200,
    EWarn  = 300,
    EError = 400
};

HUSSAR_CPU_GPU inline void Log(ELogLevel level, const char *fmt, ...) __attribute__((format (printf, 2, 3)));
HUSSAR_CPU_GPU inline void Log(ELogLevel level, const char *fmt, ...) {
    if (level <= EDebug)
        return;
    
    switch (level) {
    case ETrace: printf("[Trace]: "); break;
    case EDebug: printf("[Debug]: "); break;
    case EInfo:  printf("[Info ]: "); break;
    case EWarn:  printf("[Warn ]: "); break;
    case EError: printf("[Error]: "); break;
    }
    
#ifndef __CUDACC__
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\n");

    fflush(stdout);

    if (level >= EError) {
        assert(false);
        exit(-1);
    }
#endif
}

#ifdef NDEBUG
#define Assert(cond, msg) ((void) 0)
#else
#ifndef __CUDACC__
#define Assert(cond, msg) do { \
    if (!(cond)) \
        Log(EError, "%s in %s:%i", (msg), __FILE__, __LINE__); \
} while (0)
#else
#define Assert(cond, msg) do { \
    if (!(cond)) \
        printf("assertion failure: %s in %s:%i", (msg), __FILE__, __LINE__); \
} while (0)
#endif
#endif

}

#endif
