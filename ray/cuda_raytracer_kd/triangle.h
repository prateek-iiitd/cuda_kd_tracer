//triangle.h


#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include "ray.h"
#include <cuda_runtime.h>
#include <helper_math.h>
#include "material.h"
#include "camera.h"
#include "point_light.h"

//class PointLight;

class Triangle
{
private:
    float3 a, b, c;
    Material material;
    float3 normal_a, normal_b, normal_c;

public:
    __device__ __host__ Triangle() {}
    __device__ __host__ Triangle(const float3 _a, const float3 _b, const float3 _c):
    a(_a), b(_b), c(_c)
    {
        material = Material();
        normal_a = normalize(cross(b-a, c-a));
        normal_b = normal_a;
        normal_c = normal_a;

    }
    __device__ __host__ Triangle(const float3 _a, const float3 _b, const float3 _c, Material mat):
    a(_a), b(_b), c(_c), material(mat)
    {
        normal_a = normalize(cross(b-a, c-a));
        normal_b = normal_a;
        normal_c = normal_a;
    }

    __device__ __host__ Triangle(const float3 _a, const float3 _b, const float3 _c, const float3 vn_a, const float3 vn_b, const float3 vn_c):
    a(_a), b(_b), c(_c), normal_a(vn_a), normal_b(vn_b), normal_c(vn_c)
    {
        material = Material();
    }

    __device__ __host__ void setMaterial(const Material _mat) {material = _mat;}
    __device__ __host__ Material getMaterial() const {return material;}
    __device__ __host__ float3 barycentric(const float3 pointOnTriangle) const;
    __device__ __host__ float3 getNormal(const float3 pointOnTriangle) const;
    __device__ __host__ Ray getReflectedRay(const Ray& incidentRay, float t) const;
    __device__ __host__ Ray getRefractedRay(const Ray& incidentRay, float t) const;
    __device__ __host__ float intersect(const Ray& ray) const;
    __device__ __host__ float3 shade(const Ray& ray, float t, Triangle *objects, int objects_count, PointLight *lights, int lights_count, Camera camera) const;
    __device__ __host__ bool isReflective() const {if (material.kr > 0) return true; return false;}
    __device__ __host__ bool isRefractive() const {if (material.kt > 0) return true; return false;}
    __device__ __host__ float3 getCentroid() const {return (((a + b) + c ) / 3); }

    __device__ __host__ void getExtremes(int dimension, float &min, float &max) const;

};
#endif
