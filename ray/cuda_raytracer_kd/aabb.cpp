//aabb.cpp
//
#include "aabb.h"
#include <float.h>
#include <limits.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>

__device__ __host__ bool AABB::intersect(const Ray r, float &tmin, float &tmax) const
{
    // r.dir is unit direction vector of ray
    float3 dirfrac;
    
    float rx = r.getDirection().x;
    float ry = r.getDirection().y;
    float rz = r.getDirection().z;
    
    if (rx == 0.0)
    {
    	dirfrac.x = FLT_MAX;
    }
    else
    {
    	dirfrac.x = 1.0f / rx;
    }
    
    if (ry == 0.0)
    {
    	dirfrac.y = FLT_MAX;
    }
    else
    {
    	dirfrac.y = 1.0f / ry;
    }
    
    if (rz == 0.0)
    {
    	dirfrac.z = FLT_MAX;
    }
    else
    {
    	dirfrac.z = 1.0f / rz;
	}
	
    // bl is the corner of AABB with minimal coordinates - left bottom, tr is maximal corner
    float3 bl = bottom_left;
    float3 tr = top_right;
    
    // r.getOrigin() is origin of ray
    float t1 = (bl.x - r.getOrigin().x)*dirfrac.x;
    float t2 = (tr.x - r.getOrigin().x)*dirfrac.x;
    float t3 = (bl.y - r.getOrigin().y)*dirfrac.y;
    float t4 = (tr.y - r.getOrigin().y)*dirfrac.y;
    float t5 = (bl.z - r.getOrigin().z)*dirfrac.z;
    float t6 = (tr.z - r.getOrigin().z)*dirfrac.z;

    tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
    tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0)
    {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        return false;
    }
    return true;
}
