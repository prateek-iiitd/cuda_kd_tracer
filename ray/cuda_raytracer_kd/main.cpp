#include <iostream>
#include "triangle.h"
#include "material.h"
//#include "camera.h"
#include "ray.h"
//#include "point_light.h"
#include <limits.h>
#include <float.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_math.h>
#include <IL/il.h>
#include <IL/ilu.h>
#include <time.h>
#include "kdtracer.cu"
#include "obj_parser.h"
#include <string>
#include <vector>
#include "aabb.h"
#include "node.h"
#include <algorithm>
#include <limits>
#define MAXTRIANGLES 25

using namespace std;

int object_count = 0;
int object_list_size = 1;
int image_height = 800 * 1;
int image_width = 600 * 1;

bool compare_x (Triangle i,Triangle j)
{
    float3 centroid_i = i.getCentroid();
    float3 centroid_j = j.getCentroid();

    return centroid_i.x < centroid_j.x;
}

bool compare_y (Triangle i,Triangle j)
{
    float3 centroid_i = i.getCentroid();
    float3 centroid_j = j.getCentroid();

    return centroid_i.y < centroid_j.y;
}

bool compare_z (Triangle i,Triangle j)
{
    float3 centroid_i = i.getCentroid();
    float3 centroid_j = j.getCentroid();

    return centroid_i.z < centroid_j.z;
}



vector<Node> nodesList;
vector<Triangle> kdObjectsList;
int levelNodes[25]={0};

int constructKdTree(AABB box, vector<Triangle> objects, int dimension, int depth)
{
/*    if (depth==6 || depth==7)
    {
        cout << "Depth: " << depth << ", Size: " << objects.size() << endl;
    }
*/
    levelNodes[depth] += objects.size();

    // Create a new node and add it to the nodesList. Remember the index at which it has been added.
    Node node(box, dimension);
    int index = nodesList.size();
    nodesList.push_back(node);

    // Check if leaf criteria met or not.
    if (objects.size() > MAXTRIANGLES)
    {
        // This will be an internal node.
        //
        // Sort the triangles on the basis of their centroids in the given dimension.
        
        if (dimension == 0) { sort(objects.begin(), objects.end(), compare_x);}
        else if (dimension == 1) { sort(objects.begin(), objects.end(), compare_y);}
        else if (dimension == 2) { sort(objects.begin(), objects.end(), compare_z);}

        // Median is the centre of the sorted vector.
        int median_index = objects.size() / 2;

        // Get the splitPoint value in the given dimension.
        // Calculate bottom_left and top_right for the children.
        float splitPoint;
        float3 bl, tr;
        if (dimension == 0)
        {
            splitPoint = objects[median_index].getCentroid().x;
            bl = make_float3(splitPoint, box.bottom_left.y, box.bottom_left.z);
            tr = make_float3(splitPoint, box.top_right.y, box.top_right.z);
        }
        else if (dimension == 1) 
        {
            splitPoint = objects[median_index].getCentroid().y;
            bl = make_float3(box.bottom_left.x, splitPoint, box.bottom_left.z);
            tr = make_float3(box.top_right.x, splitPoint, box.top_right.z);
        }
        else if (dimension == 2) 
        {
            splitPoint = objects[median_index].getCentroid().z;
            bl = make_float3(box.bottom_left.x, box.bottom_left.y, splitPoint);
            tr = make_float3(box.bottom_left.x, box.top_right.y, splitPoint);
        }

        // Set this as the splitPoint for the current node.
        nodesList[index].splitPoint = splitPoint;

        vector<Triangle> leftObjects, rightObjects;

        // Iterate over the object list and add objects in their lists.
        for (int i=0; i < objects.size(); i++)
        {
            // Get min and max values in dimension for this triangle.
            float min, max;
            objects[i].getExtremes(dimension, min, max);

            // Check if triangles crosses splitting plane.
            if (min < splitPoint && max > splitPoint)  // Triangles crosses plane.
            {
                // Add triangle to both the left and right child triangles lists.
                leftObjects.push_back(objects[i]);
                rightObjects.push_back(objects[i]);
            }

            // Object in the left half only.
            else if (max < splitPoint) { leftObjects.push_back(objects[i]); }
            
            // Object in the right half only.
            else { rightObjects.push_back(objects[i]); }
        }

        int nextDim = dimension + 1;
        if (nextDim == 3) {nextDim = 0;}

        float lsize = (float)leftObjects.size();
        float rsize = (float)rightObjects.size();

        if (lsize / objects.size() > 0.75 or rsize / objects.size() > 0.75)
        {
            nodesList[index].startIndex = kdObjectsList.size();
            // Add all objects into the kdObjectsList
            kdObjectsList.insert(kdObjectsList.end(), objects.begin(), objects.end());
            nodesList[index].endIndex = kdObjectsList.size();
            vector<Triangle>().swap(objects);
 
        }
        
        else
        {
        // Recursively construct left and right subtrees.
            AABB leftBox(box.bottom_left, tr);
            AABB rightBox(bl, box.top_right);
            int lci = constructKdTree(leftBox, leftObjects, nextDim, depth+1);
            int rci = constructKdTree(rightBox, rightObjects, nextDim, depth+1);

            nodesList[index].leftChildIndex = lci;
            nodesList[index].rightChildIndex = rci;
        }
        
    }

    // Leaf node
    else
    {
        nodesList[index].startIndex = kdObjectsList.size();
        // Add all objects into the kdObjectsList
        kdObjectsList.insert(kdObjectsList.end(), objects.begin(), objects.end());
        nodesList[index].endIndex = kdObjectsList.size();
    }
    return index;
}

// Copied from Julia Assignment
void saveImage(int width, int height, unsigned char *bitmap)
{
    ILuint imageID = ilGenImage();
    ilBindImage(imageID);
    ilTexImage(width, height, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, bitmap);
    ilEnable(IL_FILE_OVERWRITE);
    char imageName[256];
    sprintf(imageName, "images/test.png");
    ilSave(IL_PNG, imageName);
    fprintf(stderr, "Image saved as %s\n", imageName);
}

Triangle* add_object(Triangle **h_objects, Triangle obj)
{
    if (object_count == object_list_size)
    {
        *h_objects = (Triangle *) realloc (*h_objects, 2 * object_list_size * sizeof(Triangle));
        object_list_size = object_list_size * 2;
    }

    (*h_objects)[object_count] = obj;
    object_count++;
    return *h_objects;
}

float3 random_float3()
{
	float one = (float) (rand() % 100) - 50;
	float two = (float) (rand() % 100) - 50;
	float three = (float) (rand() % 100) - 50;
	return make_float3(one, two, 0.0);
}

void printGPUClockFrequency()
{
    int default_device;    
    // Print clock frequency of the GPU.
    cudaGetDevice(&default_device);
    cudaDeviceProp properties;
    cudaGetDeviceProperties(&properties, default_device); 
    cout << "Clock Rate: " << properties.clockRate << endl;   
}

void setCudaStackSize(int stackSize)
{
    size_t myStackSize = stackSize;
    cudaError_t code = cudaDeviceSetLimit(cudaLimitStackSize, myStackSize); 

}

void printCurrentStackSize()
{
    size_t stackSize;
    cudaDeviceGetLimit(&stackSize, cudaLimitStackSize);
    cout << "Current Stack Size: " << stackSize << endl;
}

int main()
{
//    printGPUClockFrequency();
    cout << numeric_limits<int>::max() << endl;
    

    cudaSetDevice(0);

    printCurrentStackSize();

    setCudaStackSize(8192 * 2);
   
    srand(time(NULL));
    ilInit();

    Triangle *h_objects, *d_objects;
    Node *h_nodesList, *d_nodesList;
    PointLight *d_lights;
    vector<PointLight> h_lights;

    h_objects = (Triangle *) malloc (sizeof(Triangle));

    float3 camera_position, camera_target, camera_up;
    camera_position = make_float3(2.5, 185.0, 50.0);
    camera_target = make_float3(0.0, 180.0, 0.0);
    camera_up = make_float3(0.0, 1.0, 0.0);
    float camera_fovy = 75.0;
    Camera camera(camera_position, camera_target, camera_up, camera_fovy, image_width, image_height);

    PointLight light1 = PointLight(make_float3(45, 350.0, 65.0), make_float3(1.0, 1.0, 1.0));
    //PointLight light2 = PointLight(camera_position, make_float3(1.0, 1.0, 1.0));

    h_lights.push_back(light1);
	//h_lights.push_back(light2);

    float3 bl, tr;
    string inpFile("objs/Batman.obj");
    vector<Triangle> triangles_parsed = parse_file(inpFile, bl, tr);

    AABB sceneBox(bl, tr);

    cout << "Triangles :" << triangles_parsed.size() << endl;

//    h_objects = &triangles_parsed[0];
//    object_count = triangles_parsed.size();

    int rootIndex = constructKdTree(sceneBox, triangles_parsed, 0, 0);

//    cout << "Root Index: " << rootIndex << endl;
//    cout << "Root-left - Left Child Index: " << nodesList[rootIndex+1].leftChildIndex << endl;
//    cout << "Root-left - Right Child Index: " << nodesList[rootIndex+1].rightChildIndex << endl;
//    cout << "Root - SplitDimension: " << nodesList[rootIndex+1].splitDimension << endl;
//    cout << "Root - SplitPoint: " << nodesList[rootIndex].splitPoint << endl;
    
    for (int i=0; i<nodesList.size(); i++)
    {
        if (nodesList[i].leftChildIndex < 0 || nodesList[i].rightChildIndex < 0 )
        {
        	if (nodesList[i].startIndex < 0 || nodesList[i].endIndex < 0)
        	{
		cout << "Under" << endl;
            	cout << "index: " << i << endl;
            	cout << "Left Index: " << nodesList[i].leftChildIndex<< endl;
            	cout << "Right Index: " << nodesList[i].rightChildIndex << endl;
            	cout << "Start Index: " << nodesList[i].startIndex << endl;
            	cout << "End Index: " << nodesList[i].endIndex << endl << endl;
            }
        }
        if ((int)nodesList[i].leftChildIndex > (int)nodesList.size()-1 || (int)nodesList[i].rightChildIndex > (int)nodesList.size()-1 )
        {
		cout << "Over(l/r)" << endl;
            	cout << "index: " << i << endl;
		cout << "NodeListSize" << nodesList.size() << endl;
            	cout << "Left Index: " << nodesList[i].leftChildIndex<< endl;
            	cout << "Right Index: " << nodesList[i].rightChildIndex<< endl;
            	cout << "Start Index: " << nodesList[i].startIndex << endl;
            	cout << "End Index: " << nodesList[i].endIndex << endl << endl;
        }
        if ((int)nodesList[i].startIndex > (int)kdObjectsList.size()-1 || (int)nodesList[i].endIndex > (int)kdObjectsList.size() )
        {
		cout << "Over(s/e)" << endl;
            	cout << "index: " << i << endl;
		cout << "kdOListSize" << kdObjectsList.size() << endl;
            	cout << "Left Index: " << nodesList[i].leftChildIndex<< endl;
            	cout << "Right Index: " << nodesList[i].rightChildIndex<< endl;
            	cout << "Start Index: " << nodesList[i].startIndex<< endl;
            	cout << "End Index: " << nodesList[i].endIndex<< endl << endl;
        }

    }

    cout << "kdObjectsList (Size): " << kdObjectsList.size() << endl;
    cout << "nodesList (Size): " << nodesList.size() << endl;

    for (int i=0; i<25; i++)
    {
        cout << "Level" << i << ": " << levelNodes[i] << endl;
    }

    h_objects = &kdObjectsList[0];
    object_count = kdObjectsList.size();
    
    h_nodesList = &nodesList[0];

    /*    Material boxMat;
    boxMat.color = make_float3(0.1, 0.0, 0.3);
    boxMat.kr = 0.4;

    for (int i=triangles_parsed.size() - 1; i >= triangles_parsed.size() - 28; i--)
    {
        triangles_parsed[i].setMaterial(boxMat);
    }
*/
    
    cudaMalloc((void**)&d_objects, object_count * sizeof(Triangle));
    cudaMemcpy(d_objects, h_objects, object_count * sizeof(Triangle), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&d_nodesList, nodesList.size() * sizeof(Triangle));
    cudaMemcpy(d_nodesList, h_nodesList, nodesList.size() * sizeof(Triangle), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&d_lights, h_lights.size() * sizeof(PointLight));
    cudaMemcpy(d_lights, &h_lights[0], h_lights.size() * sizeof(PointLight), cudaMemcpyHostToDevice);

    // Copy constants
    cudaMemcpyToSymbol(cam, &camera, sizeof(Camera));
    cudaMemcpyToSymbol(c_image_height, &image_height, sizeof(int));
    cudaMemcpyToSymbol(c_image_width, &image_width, sizeof(int));

    unsigned char *h_image = new unsigned char[image_height * image_width * 3];
    memset(h_image, 0, image_height * image_width * 3 * sizeof(char));
    
    long long int *h_time = new long long int[image_height * image_width];
    memset(h_time, 0, image_height * image_width * sizeof(long long int));
    
    long long int *d_time;
    cudaMalloc((void**)&d_time, image_height * image_width * sizeof(long long int));
    cudaMemcpy(d_time, h_time, image_height * image_width * sizeof(long long int), cudaMemcpyHostToDevice);

    //saveImage(image_width, image_height, h_image);

    unsigned char *d_image;
    cudaMalloc((void**)&d_image, image_height * image_width * 3 * sizeof(char));
    cudaMemcpy(d_image, h_image, image_height * image_width * 3 * sizeof(char), cudaMemcpyHostToDevice);

    dim3 block, grid;
    block.x = 8;
    block.y = 8;
    grid.x = image_width / block.x;
    grid.y = image_height / block.y;
    
    //Call to kernel
    colorPixel<<<grid, block>>>(d_image, d_objects, object_count, d_lights, h_lights.size(), d_time, d_nodesList);
    cudaDeviceSynchronize();
    cudaError_t errCode = cudaGetLastError();

    if (cudaSuccess != errCode)
    {
        cout << cudaGetErrorString(errCode) << endl;
    }
    else
    {
        cout << "Success!"  << endl;
    }
    
    // Copy results
    cudaMemcpy(h_image, d_image, image_height * image_width * 3 * sizeof(char), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_time, d_time, image_height * image_width * sizeof(long long int), cudaMemcpyDeviceToHost);
   
    // Save the image
    saveImage(image_width, image_height, h_image);

    ofstream outFile;
    outFile.open ("times.csv", std::ofstream::out | std::ofstream::trunc);

    for (int y=0; y<image_height; y++)
    {
        for (int x=0; x<image_width; x++)
        {
            outFile << h_time[y*image_width + x] << ", ";
        }
        outFile << endl;
    }
    outFile.close();

    cudaFree(d_image);
    delete[] h_image;
}
