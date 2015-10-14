// aabb.h
//
#ifndef _AABB_H_
#define _AABB_H_

#include "ray.h"
#include <cuda_runtime.h>
#include <helper_math.h>

class AABB
{
public:
    float3 bottom_left, top_right;

	__device__ __host__ AABB() {}
    __device__ __host__ AABB(const float3 _bl, const float3 _tr):
        bottom_left(_bl), top_right(_tr) {}

    __device__ __host__ bool intersect(const Ray r, float &tmin, float &tmax) const;
};

#endif
