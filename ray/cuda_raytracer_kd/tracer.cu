#include <cuda_runtime.h>
#include <helper_math.h>
#include <helper_cuda.h>
#include <float.h>
#include "ray.h"
#include "triangle.h"
#include "camera.h"

#define MAX_DEPTH 2
#define SMALLEST_DIST 1e-4

__constant__ Camera cam;
__constant__ int c_image_width;
__constant__ int c_image_height;

__device__ void	writePixel(int x, int y, int image_width, float3 color, unsigned char *d_image);

__device__ float3 traceRay(Ray& ray, Triangle *d_objects, int objects_count, PointLight *lights, int lights_count)
{
    // Exit if MAX_DEPTH reached.
	if (ray.getDepth() > MAX_DEPTH)
	{
		return make_float3(0.0, 0.0, 0.0);
	}
	
	else
	{
		float tmin = FLT_MAX;
		Triangle closestObject;
		float t;
		
        // Find closest hit object.
		for (int i=0; i<objects_count; i++)
		{
			t = d_objects[i].intersect(ray);
			
			if (t < tmin && t > SMALLEST_DIST)
			{
				tmin  = t;
				closestObject = d_objects[i];
			}
			
		}
		
        // Some object was hit.
        if (tmin != FLT_MAX)
        {
		    float3 final_color =  closestObject.shade(ray, tmin, d_objects, objects_count, lights, lights_count, cam); 
            if (closestObject.isReflective())
            {
                Ray reflectedRay = closestObject.getReflectedRay(ray, tmin);
                float3 r_color = closestObject.getMaterial().kr * traceRay(reflectedRay, d_objects, objects_count, lights, lights_count);
                final_color = final_color + r_color;
            }
            if (closestObject.isRefractive())
            {
                Ray refractedRay = closestObject.getRefractedRay(ray, tmin);
                float3 r_color = closestObject.getMaterial().kt * traceRay(refractedRay, d_objects, objects_count, lights, lights_count);
                final_color = final_color + r_color;
            }
            return final_color;
	    }
        else
        {
            return make_float3(0.0, 0.0, 0.0);
        }
    }

}

__global__ void colorPixel(unsigned char *d_image, Triangle *d_objects, int objects_count, PointLight *lights, int lights_count, long long int *times)
{
    long long int start = clock64();
	int x = (blockIdx.x * blockDim.x) + threadIdx.x;
	int y = (blockIdx.y * blockDim.y) + threadIdx.y;
	
	if (x >= c_image_width || y >= c_image_height)
		return;
	
	float3 ray_dir = cam.get_ray_direction(x, y);
	Ray ray(cam.getPosition(), ray_dir);
	
	float3 pixel_color = traceRay(ray, d_objects, objects_count, lights, lights_count);
    long long int end = clock64();
    long long int duration = end - start;
    times[y * c_image_width + x] = duration;
    writePixel(x, y, c_image_width, pixel_color, d_image);
}


__device__ void writePixel(int x, int y, int image_width, float3 color, unsigned char *d_image)
{
	d_image[(y*image_width + x) * 3 + 0] = (unsigned char) (color.x * 255);
	d_image[(y*image_width + x) * 3 + 1] = (unsigned char) (color.y * 255);
	d_image[(y*image_width + x) * 3 + 2] = (unsigned char) (color.z * 255);
}
