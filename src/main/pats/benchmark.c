#include "benchmark.h"
#include <stdio.h>
#include <string.h>

void benchmark_start(benchmark_t* b, const char* name)
{
    strncpy(b->name, name, (BENCHMARK_MAX_STRING_LENGTH-1));
    b->name[(BENCHMARK_MAX_STRING_LENGTH-1)]='\0';
    b->start = micros();
}

void benchmark_stop(benchmark_t* b)
{
    b->stop = micros();
}

void benchmark_tostr(benchmark_t* b, char* str)
{
    char tmp[16];

    strcpy(str, "benchmark: ");
    strcat(str, b->name);
    strcat(str, " ");
    sprintf(tmp, "%-5d", b->stop - b->start);
    //itoa(b->stop - b->start, tmp, 10);
    strcat(str, tmp);
    strcat(str, " us\r\n");
}