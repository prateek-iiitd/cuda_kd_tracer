#ifndef _POINT_LIGHT_H_
#define _POINT_LIGHT_H

#include "light.h"

class PointLight : public Light
{
private:
    float3 position;

public:
    __device__ __host__ PointLight(const float3 _pos, const float3 _intensity):
        Light(_intensity), position(_pos) {}
    __device__ __host__ float3 getPosition() const {return position;}


};

#endif
