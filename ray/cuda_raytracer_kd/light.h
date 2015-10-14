#ifndef _LiGHT_H_
#define _LIGHT_H_

#include <cuda_runtime.h>
#include <helper_math.h>

class Light
{
public:
    float3 intensity;

    __device__ __host__ Light(const float3 _intensity):
        intensity(_intensity) {}


};

#endif
