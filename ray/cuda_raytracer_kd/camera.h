#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>

class Camera
{
private:
    float3 target, up, position;
    float3 line_of_sight;
    float3 u, v, w; //Camera basis vectors
    float fovy, aspect;
    float focalHeight, focalWidth, focalDistance; 
    int height, width;

public:
    __device__ __host__ Camera() {};
    __device__ __host__ Camera(const float3 _pos, const float3 _target, const float3 _up, float _fovy, int _w, int _h);
    __device__ __host__ const float3 get_ray_direction(const int i, const int j) const;
    __device__ __host__ const float3 getPosition() const { return position; }
    __device__ __host__ int getWidth() {return width;}
    __device__ __host__ int getHeight(){return height;}
    
};

#endif
