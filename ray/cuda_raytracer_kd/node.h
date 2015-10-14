// node.h
//
#ifndef _NODE_H_
#define _NODE_H_

#include <cuda_runtime.h>
#include <helper_math.h>

#include "aabb.h"

class Node
{
public:
    AABB boundingBox;
    int splitDimension;
    float splitPoint;
    int leftChildIndex, rightChildIndex; // Indices that point to the KD tree left/right children.
    int startIndex, endIndex; // Indices that point to the triangles list array.

	__host__ __device__ Node() {}

    __host__ __device__ Node(AABB _box, int _dim):
        boundingBox(_box), splitDimension(_dim)
    {
        startIndex = -1; endIndex = -1;
        leftChildIndex = -1; rightChildIndex = -1;
        splitPoint = 0.0;
    }
};

#endif
