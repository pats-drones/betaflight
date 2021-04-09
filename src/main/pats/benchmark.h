#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BENCHMARK_MAX_STRING_LENGTH 32

enum EBenchmarkState 
{
    E_BENCHMARK_STATE_NONE,
    E_BENCHMARK_STATE_RUNNING,
    E_BENCHMARK_STATE_STOPPED,
    E_BENCHMARK_STATE_COMPLETED
};

typedef struct 
{
    uint32_t start;
    uint32_t stop;
    char name[BENCHMARK_MAX_STRING_LENGTH];
} benchmark_t;

void benchmark_start(benchmark_t* b, const char* name);
void benchmark_stop(benchmark_t* b);
void benchmark_tostr(benchmark_t* b, char* str);

#ifdef __cplusplus
}
#endif

#endif // BENCHMARK_H_