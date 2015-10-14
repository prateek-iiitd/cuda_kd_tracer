#include <cuda_runtime.h>
#include <helper_math.h>
#include <helper_cuda.h>
#include <float.h>
#include "ray.h"
#include "triangle.h"
#include "camera.h"
#include "aabb.h"
#include "node.h"

#define MAX_DEPTH 0
#define SMALLEST_DIST 1e-4

__constant__ Camera cam;
__constant__ int c_image_width;
__constant__ int c_image_height;

__device__ void	writePixel(int x, int y, int image_width, float3 color, unsigned char *d_image);
__device__ float planeIntersection(Ray ray, int dimension, float splitPoint)
{
	float *origin = &(ray.getOrigin().x);
	float *direction  = &(ray.getDirection().x); 
	
	if (direction[dimension] == 0.0) {return FLT_MAX; }
	
	float t = (splitPoint - origin[dimension]) / direction[dimension];
	return t;
}

__device__ float closestHitObject(Ray& ray, Triangle *d_objects, Node *d_nodesList, Triangle &closestObject, long long int &intersectionTests)
{
    Node root = d_nodesList[0];
    float tmin, tmax;

    bool intersection = root.boundingBox.intersect(ray, tmin, tmax);

    // No intersection with the scene bounding box. Return immediately.
    if (!intersection)
    {
        return FLT_MAX;
    }

    else
    {
        float tclosest = FLT_MAX;
        Node nearChild, farChild;
        
        // Initialize stack to hold traversal nodes.
        int stackPosition = 0;
        //int stackMemory = 16;
        // Node *stack = (Node *) malloc (stackMemory * sizeof(Node));
		Node stack[24];


        // Find intersection point with splitting plane.
        float tPlane = planeIntersection(ray, root.splitDimension, root.splitPoint);
        
        // Compute value along splitting dimension for plane, and tmin, tmax.
        float3 tPlanePoint = ray.getOrigin() + tPlane * ray.getDirection();
        float tPlaneDimPoint = (&(tPlanePoint.x))[root.splitDimension];
        
        float3 tMinPoint = ray.getOrigin() + tmin * ray.getDirection();
        float tMinDimPoint = (&(tMinPoint.x))[root.splitDimension];

        float3 tMaxPoint = ray.getOrigin() + tmax * ray.getDirection();
        float tMaxDimPoint = (&(tMaxPoint.x))[root.splitDimension];

        // Both children are intersected by the ray.
        if (0 < tPlane && tPlane < tmax)
        {
            // Classify the near and far children for the ray.
            if (tMinDimPoint < tPlaneDimPoint)
            {
                nearChild = d_nodesList[root.leftChildIndex];
                farChild = d_nodesList[root.rightChildIndex];
            }
            else
            {
                farChild = d_nodesList[root.leftChildIndex];
                nearChild = d_nodesList[root.rightChildIndex];
            }
            stack[stackPosition++] = farChild;
            stack[stackPosition++] = nearChild;
        }

        // Only one child is intersected.
        else if (tMinDimPoint < tPlaneDimPoint)
        {
            nearChild = d_nodesList[root.leftChildIndex];
            stack[stackPosition++] = nearChild;
        }
        else
        {
            nearChild = d_nodesList[root.rightChildIndex];
            stack[stackPosition++] = nearChild;
        }


        // Iterative traversal of the kdtree using the stack.
        while(stackPosition!=0)
        {
            Node currentNode = stack[stackPosition-1];
            stackPosition--;

/*            if (4 * stackPosition < stackMemory && stackMemory > 8)
            {
                stack = (Node *) realloc(stack, stackMemory * sizeof(Node) / 2);
                stackMemory = stackMemory / 2;
            }
*/
            // if not a leaf
            if(currentNode.leftChildIndex != -1)
            {
                // Find intersection point with splitting plane.
                tPlane = planeIntersection(ray, currentNode.splitDimension, currentNode.splitPoint);
        
                // Compute value along splitting dimension for plane, and tmin, tmax.
                tPlanePoint = ray.getOrigin() + tPlane * ray.getDirection();
                tPlaneDimPoint = (&(tPlanePoint.x))[currentNode.splitDimension];
        
                tMinPoint = ray.getOrigin() + tmin * ray.getDirection();
                tMinDimPoint = (&(tMinPoint.x))[currentNode.splitDimension];

                tMaxPoint = ray.getOrigin() + tmax * ray.getDirection();
                tMaxDimPoint = (&(tMaxPoint.x))[currentNode.splitDimension];

                // Both children are intersected by the ray.
                if (0 < tPlane && tPlane < tmax)
                {
                    // Classify the near and far children for the ray.
                    if (tMinDimPoint < tPlaneDimPoint)
                    {
                        nearChild = d_nodesList[currentNode.leftChildIndex];
                        farChild = d_nodesList[currentNode.rightChildIndex];
                    }
                    else
                    {
                        farChild = d_nodesList[currentNode.leftChildIndex];
                        nearChild = d_nodesList[currentNode.rightChildIndex];
                    }
                    // Allocate more memory in the stack if needed.
/*                    if (stackPosition == stackMemory)
                    {
                        stack = (Node *) realloc (stack, 2 * stackMemory * sizeof(Node));
                        stackMemory = stackMemory * 2;
                    }
*/
                    stack[stackPosition++] = farChild;
                    stack[stackPosition++] = nearChild;
                }

                // Only one child is intersected.
                else if (tMinDimPoint < tPlaneDimPoint)
                {
                    Node nearChild = d_nodesList[currentNode.leftChildIndex];
                    // Allocate more memory in the stack if needed.
/*                    if (stackPosition == stackMemory)
                    {
                        stack = (Node *) realloc (stack, 2 * stackMemory * sizeof(Node));
                        stackMemory = stackMemory * 2;
                    }
*/
                    stack[stackPosition++] = nearChild;
                }
                else
                {
                    Node nearChild = d_nodesList[currentNode.rightChildIndex];
                    // Allocate more memory in the stack if needed.

/*                    if (stackPosition == stackMemory)
                    {
                        stack = (Node *) realloc (stack, 2 * stackMemory * sizeof(Node));
                        stackMemory = stackMemory * 2;
                    }
*/
                    stack[stackPosition++] = nearChild;
                }
               
            }
            // Leaf node
            else
            {
                tclosest = FLT_MAX;
                float t;
                // Intersect ray with all triangles in the leaf. Find closest triangle. Return if any intersection is found.
                for (int i=currentNode.startIndex; i < currentNode.endIndex; i++)
                {
                    t = d_objects[i].intersect(ray);
                    if (t < tclosest)
                    {
                        tclosest = t;
                        closestObject = d_objects[i];
                        stackPosition = 0;
                    }
                }
                intersectionTests += (currentNode.endIndex - currentNode.startIndex ) - 1;
            }
        }
        
        return tclosest;
        
    }


}

__device__ float3 traceRay(Ray& ray, Triangle *d_objects, int objects_count, PointLight *lights, int lights_count,  Node *d_nodesList, long long int &intersections)
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
		
        // Find closest hit object.
		tmin = closestHitObject(ray, d_objects, d_nodesList, closestObject, intersections);
		
        // Some object was hit.
        if (tmin != FLT_MAX)
        {
//            float3 final_color = make_float3(1.0, 1.0, 0.0);
		    float3 final_color =  closestObject.shade(ray, tmin, d_objects, objects_count, lights, lights_count, cam); 
/*            if (closestObject.isReflective())
            {
                Ray reflectedRay = closestObject.getReflectedRay(ray, tmin);
                float3 r_color = closestObject.getMaterial().kr * traceRay(reflectedRay, d_objects, objects_count, lights, lights_count, d_nodesList);
                final_color = final_color + r_color;
            }
            if (closestObject.isRefractive())
            {
                Ray refractedRay = closestObject.getRefractedRay(ray, tmin);
                float3 r_color = closestObject.getMaterial().kt * traceRay(refractedRay, d_objects, objects_count, lights, lights_count, d_nodesList);
                final_color = final_color + r_color;
            }

*/            return final_color;
	    }
        else
        {
            return make_float3(0.0, 0.0, 0.0);
        }
    }

}

__global__ void colorPixel(unsigned char *d_image, Triangle *d_objects, int objects_count, PointLight *lights, int lights_count, long long int *times, Node *d_nodesList)
{
    //long long int start = clock64();
	int x = (blockIdx.x * blockDim.x) + threadIdx.x;
	int y = (blockIdx.y * blockDim.y) + threadIdx.y;
	
	if (x >= c_image_width || y >= c_image_height)
		return;
	
	float3 ray_dir = cam.get_ray_direction(x, y);
	Ray ray(cam.getPosition(), ray_dir);
	
    long long int intersections = 0;
	float3 pixel_color = traceRay(ray, d_objects, objects_count, lights, lights_count, d_nodesList, intersections);
    //long long int end = clock64();
    //long long int duration = end - start;
    times[y * c_image_width + x] = intersections;
    writePixel(x, y, c_image_width, pixel_color, d_image);
}


__device__ void writePixel(int x, int y, int image_width, float3 color, unsigned char *d_image)
{
	d_image[(y*image_width + x) * 3 + 0] = (unsigned char) (color.x * 255);
	d_image[(y*image_width + x) * 3 + 1] = (unsigned char) (color.y * 255);
	d_image[(y*image_width + x) * 3 + 2] = (unsigned char) (color.z * 255);
}
