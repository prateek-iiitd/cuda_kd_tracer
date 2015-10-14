/* Reference:
http://stackoverflow.com/questions/236129/split-a-string-in-c/236803#236803
*/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <helper_math.h>
#include <helper_cuda.h>
#include "triangle.h"

using namespace std;

// trim from left
inline std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v")
{
    s.erase(0, s.find_first_not_of(t));
    return s;
}

// trim from right
inline std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v")
{
    s.erase(s.find_last_not_of(t) + 1);
    return s;
}

// trim from left & right
inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v")
{
    return ltrim(rtrim(s, t), t);
}

vector<string> &split(const string &s, char delim, vector<string> &elems)
{
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        if (!item.empty() || delim=='/') {
            elems.push_back(item);
        }
    }
    return elems;
}


vector<string> split(const string &s, char delim)
{
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}


vector<Triangle> parse_file(const string filename, float3 &bl, float3 &tr)
{
    ifstream infile(filename.c_str());

    vector<float3> vertices;
    vector<Triangle> triangles;
    vector<float3> normals;

    string line;
    int line_number = 0;

    string vertex("v");
    string face("f");
    string normal("vn");
    
    bl = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
    tr = make_float3(FLT_MIN, FLT_MIN, FLT_MIN);

    while (getline(infile, line))
    {
        line = rtrim(line);
        line_number += 1;
        vector<string> elems;
        split(line, ' ', elems);

        if (elems.size()==0)
            continue;

        if (elems[0].compare(vertex)==0)
        {
            if (elems.size() != 4)
            {
                fprintf(stderr, "Invalid line %d\n. Exiting!", line_number);
                exit(0);
            }

            else
            {
                try {
                    float a = atof(elems[1].c_str());
                    float b = atof(elems[2].c_str());
                    float c = atof(elems[3].c_str());
                    
                    if (a < bl.x) {bl.x = a; }
                    if (a > tr.x) {tr.x = a; }

                    if (b < bl.y) {bl.y = b; }
                    if (b > tr.y) {tr.y = b; }

                    if (c < bl.z) {bl.z = c; }
                    if (c > tr.z) {tr.z = c; }

                    vertices.push_back(make_float3(a, b, c));

                }
                catch (...) {
                    fprintf(stderr, "Invalid line %d\n. Exiting!", line_number);
                    exit(0);
                }
            }
        }

        else if (elems[0].compare(face)==0)
        {
            if (elems.size() != 4 && elems.size()!=5)
            {
                fprintf(stderr, "1.Invalid line %d\n. Exiting!", line_number);
                exit(0);
            }

            if (elems.size()==4)
            {   
                try {
//                        float3 a = vertices[atoi(elems[1].c_str()) - 1];
//                        float3 b = vertices[atoi(elems[2].c_str()) - 1];
//                        float3 c = vertices[atoi(elems[3].c_str()) - 1];
//                        triangles.push_back(Triangle(a, b, c));
                        vector<string> first_elems = split(elems[1], '/');
                        vector<string> second_elems = split(elems[2], '/');
                        vector<string> third_elems = split(elems[3], '/');
                        
                        float3 a = vertices[atoi(first_elems[0].c_str()) - 1];
                        float3 b = vertices[atoi(second_elems[0].c_str()) - 1];
                        float3 c = vertices[atoi(third_elems[0].c_str()) - 1];

                        float3 vn_a = normals[atoi(first_elems[2].c_str()) - 1];
                        float3 vn_b = normals[atoi(second_elems[2].c_str()) - 1];
                        float3 vn_c = normals[atoi(third_elems[2].c_str()) - 1];

                        triangles.push_back(Triangle(a, b, c, vn_a, vn_b, vn_c));
//                        triangles.push_back(Triangle(a, b, c));
                }
                catch (...) {
                    fprintf(stderr, "2.Invalid line %d\n. Exiting!", line_number);
                    exit(0);
                }
            }

            else
            {
                try {
//                        float3 a = vertices[atoi(elems[1].c_str()) - 1];
//                        float3 b = vertices[atoi(elems[2].c_str()) - 1];
//                        float3 c = vertices[atoi(elems[3].c_str()) - 1];
//                        triangles.push_back(Triangle(a, b, c));
                        vector<string> first_elems = split(elems[1], '/');
                        vector<string> second_elems = split(elems[2], '/');
                        vector<string> third_elems = split(elems[3], '/');
                        vector<string> fourth_elems = split(elems[4], '/');

                        float3 a = vertices[atoi(first_elems[0].c_str()) - 1];
                        float3 b = vertices[atoi(second_elems[0].c_str()) - 1];
                        float3 c = vertices[atoi(third_elems[0].c_str()) - 1];
                        float3 d = vertices[atoi(fourth_elems[0].c_str()) - 1];

                        float3 vn_a = normals[atoi(first_elems[2].c_str()) - 1];
                        float3 vn_b = normals[atoi(second_elems[2].c_str()) - 1];
                        float3 vn_c = normals[atoi(third_elems[2].c_str()) - 1];
                        float3 vn_d = normals[atoi(fourth_elems[2].c_str()) - 1];

                        triangles.push_back(Triangle(a, b, c, vn_a, vn_b, vn_c));
                        triangles.push_back(Triangle(a, c, d, vn_a, vn_c, vn_d));
                }
                catch (...) {
                    fprintf(stderr, "Invalid line %d\n. Exiting!", line_number);
                    exit(0);
                }
 
            }
        }
        
        else if (elems[0].compare(normal)==0)
        {
            if (elems.size() != 4)
            {
                fprintf(stderr, "Invalid line %d\n. Exiting!", line_number);
                exit(0);
            }

            else
            {
                try {
                    normals.push_back(make_float3(atof(elems[1].c_str()), atof(elems[2].c_str()), atof(elems[3].c_str())));
                }
                catch (...) {
                    fprintf(stderr, "Invalid line %d\n. Exiting!", line_number);
                    exit(0);
                }
            }

        }
    }   

    return triangles;
}

