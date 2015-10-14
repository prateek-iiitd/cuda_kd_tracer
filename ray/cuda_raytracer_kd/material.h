//material.h

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>

class Material
{
public:
    float3 color;
    float ka;//Ambient contribution
    float kd;//Diffuse constant
    float ks;//Specular constant
    float kr;//Contribution from reflection
    float kt;//Contribution from refraction
    float refractive_index;//Coefficient of refraction
    float n;//Phong's shiny constatnt

    __device__ __host__ Material():
        color(make_float3(0.9, 0.9, 0.9)), ka(0.05), kd(0.9), ks(0), kr(0), kt(0), n(0), refractive_index(0) {}
};


#endif
