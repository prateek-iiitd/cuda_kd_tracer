//ray.h

#ifndef _RAY_H_
#define _RAY_H_

#include <float.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>


class Ray
{
private:
	float3 origin, direction;
	int depth;
	float refractive_index;

public:
	__device__ __host__ Ray(const float3 o, const float3 d, int _depth = 0, float _ref_idx = 1.0):
	origin(o), direction(d), depth(_depth), refractive_index(_ref_idx)
    {
    	direction = normalize(direction);
    }
    __device__ __host__ float3 getOrigin() const  {return origin;}
    __device__ __host__ float3 getDirection() const  {return direction;}
    __device__ __host__ int getDepth() const {return depth;}
    __device__ __host__ float getRefractiveIndex() const {return refractive_index;}
};
#endif
