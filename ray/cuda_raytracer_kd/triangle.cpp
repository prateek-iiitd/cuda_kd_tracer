//triangle.cpp

#include "triangle.h"
#include <float.h>
#include <limits.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>

typedef struct
{
        float3 m[3];
} float3x3;

#define SMALLEST_DIST 1e-4

__device__ __host__ void Triangle::getExtremes(int dimension, float &min, float &max) const
{
    if (dimension == 0)
    {
        min = a.x;
        max = a.x;

        if (b.x < min) {min = b.x ;}
        if (b.x > max) {max = b.x ;}

        if (c.x < min) {min = c.x ;}
        if (c.x > max) {max = c.x ;}
    }

    else if (dimension == 1)
    {
        min = a.y;
        max = a.y;

        if (b.y < min) {min = b.y ;}
        if (b.y > max) {max = b.y ;}

        if (c.y < min) {min = c.y ;}
        if (c.y > max) {max = c.y ;}
 
    }

    else if (dimension == 2)
    {
        min = a.z;
        max = a.z;

        if (b.z < min) {min = b.z ;}
        if (b.z > max) {max = b.z ;}

        if (c.z < min) {min = c.z ;}
        if (c.z > max) {max = c.z ;}
    }
}

__host__ __device__ float3 Triangle::barycentric(const float3 pointOnTriangle) const
{
    float3 bary = make_float3(0.0, 0.0, 0.0);

    float3 v0 = b - a;
    float3 v1 = c - a;
    float3 v2 = pointOnTriangle - a;


    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    bary.y = (d11 * d20 - d01 * d21) / denom;
    bary.z = (d00 * d21 - d01 * d20) / denom;
    bary.x = 1.0f - bary.y - bary.z;

    return bary;
}

__host__ __device__ float3 Triangle::getNormal(const float3 pointOnTriangle) const
{
    float3 bary = barycentric(pointOnTriangle);

    //float3 bary = make_float3(1.0, 0.0, 0.0);
    float3 interpolatedNormal = (bary.x * normal_a) + 
                                (bary.y * normal_b) + 
                                (bary.z * normal_c);

    return normalize(interpolatedNormal);
}

__host__ __device__ float determinant(const float3x3 mat)
{
    float res = 0;
    float3 row1 = mat.m[0];
    float3 row2 = mat.m[1];
    float3 row3 = mat.m[2];

    res += (row1.x * (row2.y * row3.z - row2.z * row3.y));
    res -= (row1.y * (row2.x * row3.z - row2.z * row3.x));
    res += (row1.z * (row2.x * row3.y - row2.y * row3.x));
    
    return res;
}

__device__ __host__ float Triangle::intersect(const Ray& ray) const
{
    
    float3 d = ray.getDirection();
    float3 e = ray.getOrigin();

    //Ray and triangle are parallel.

/*    if (dot(d, normal)==0)
    {
        return FLT_MAX;
    }
*/
    float3x3 aMat;
    aMat.m[0] = make_float3(a.x - b.x, a.x - c.x, d.x);
    aMat.m[1] = make_float3(a.y - b.y, a.y - c.y, d.y);
    aMat.m[2] = make_float3(a.z - b.z, a.z - c.z, d.z);
    float detA = determinant(aMat);

    float3x3 bMat;
    bMat.m[0] = make_float3(a.x - e.x, a.x - c.x, d.x);
    bMat.m[1] = make_float3(a.y - e.y, a.y - c.y, d.y);
    bMat.m[2] = make_float3(a.z - e.z, a.z - c.z, d.z);
    float detB = determinant(bMat);

    float beta = detB / detA;

    float3x3 cMat;
    cMat.m[0] = make_float3(a.x - b.x, a.x - e.x, d.x);
    cMat.m[1] = make_float3(a.y - b.y, a.y - e.y, d.y);
    cMat.m[2] = make_float3(a.z - b.z, a.z - e.z, d.z);
    float detC = determinant(cMat);

    float gamma = detC / detA;

    float3x3 tMat;
    tMat.m[0] = make_float3(a.x - b.x, a.x - c.x, a.x - e.x);
    tMat.m[1] = make_float3(a.y - b.y, a.y - c.y, a.y - e.y);
    tMat.m[2] = make_float3(a.z - b.z, a.z - c.z, a.z - e.z);
    float detT = determinant(tMat);

    if (beta > 0 && gamma > 0 && (beta + gamma)<1)
    {
        float t = detT / detA;
        return t;
    }
    else
    {
        return FLT_MAX;
    }
}

__device__ __host__ Ray Triangle::getReflectedRay(const Ray& incidentRay, float t) const
{
    float3 hitPoint = incidentRay.getOrigin() + (t * incidentRay.getDirection());

    float3 d = incidentRay.getDirection();
    float3 n = getNormal(hitPoint);

    float3 r = d - ((2.0 * dot(d, n)) * n);
    r = normalize(r);

    Ray reflectedRay(hitPoint, r, incidentRay.getDepth() + 1, incidentRay.getRefractiveIndex());
    return reflectedRay;
}


__device__ __host__ Ray Triangle::getRefractedRay(const Ray& incidentRay, float t) const
{
    float3 hitPoint = incidentRay.getOrigin() + (t * incidentRay.getDirection());

    float startRefractiveIndex = 1.0;
    float endRefractiveIndex = 1.0;

    //When the ray is entering into the medium.
    if (incidentRay.getRefractiveIndex() != material.refractive_index)
    {
        startRefractiveIndex = incidentRay.getRefractiveIndex();
        endRefractiveIndex = material.refractive_index;
    }

    else
    {
        startRefractiveIndex = material.refractive_index;
        endRefractiveIndex = 1.0;
    }

    float eta = startRefractiveIndex / endRefractiveIndex;

    float3 incident = incidentRay.getDirection();
    float3 n = getNormal(hitPoint);

    float dotProduct = dot(incident, n);

    if (dotProduct > 0)
    {
        n = -1*n;
    }

    float cosI = -1.0 * dot(n, incident);
    float cosT2 = 1.0 - eta * eta * (1.0 - cosI * cosI);
    
    if (cosT2 > 0.0)
    {
        float3 r = (eta * incident) + (eta * cosI - sqrtf(cosT2)) * n;
        r = normalize(r);
        return Ray(hitPoint, r, incidentRay.getDepth() + 1, endRefractiveIndex);
    }

    // TODO: Handle TIR
    else
    {
        return Ray(make_float3(0.0, 0.0, 0.0), make_float3(0.0, 0.0, 0.0));
    }
}

__device__ __host__ float3 Triangle::shade(const Ray& ray, float t, Triangle *objects, int objects_count, PointLight *lights, int lights_count, Camera camera) const
{
    //return make_float3(0.3, 0.3, 0.3);
    
    float3 pointIllumination = make_float3(0.0, 0.0, 0.0);

    float3 hitPoint = ray.getOrigin() + (t * ray.getDirection());

    for (int i=0; i<lights_count; i++)
    {
        // Check if point is in shadow.
        //

        bool inShadow = false;

        float3 toLight = lights[i].getPosition() - hitPoint;
        Ray shadowRay(hitPoint, toLight);

        float lightDistance = sqrtf(dot(toLight, toLight));

        /*float shadowT = 0.0;
        for (int j=0; j<objects_count; j++)
        {
            shadowT = objects[j].intersect(shadowRay);
            //float shadowT = 2.0;

            if (shadowT != FLT_MAX)
            {
//                float hitObjDistance = sqrtf(shadowT * shadowRay.getDirection());

                if (shadowT < lightDistance &&  shadowT > SMALLEST_DIST)
                {
                    inShadow = true;
                    break;
                }
            }
        }*/

        if (!inShadow)
        {

            //Compute ambient term
            float3 ambient = material.ka * lights[i].intensity;

            //Add diffuse illumination
            pointIllumination += ambient;

            //Compute diffuse term
            float3 l = normalize(toLight);
            float dotResult = dot(l, getNormal(hitPoint));
            dotResult = clamp(dotResult, 0.0, 1.0);
        
            // Add diffuse illumination
            float3 diffuse = (material.kd * dotResult) * lights[i].intensity;
            pointIllumination += diffuse;
    
        }
    }

    return pointIllumination * material.color;

}
