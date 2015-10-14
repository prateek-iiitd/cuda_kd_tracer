//camera.cpp
#include "camera.h"

__device__ __host__ Camera::Camera(float3 _pos, float3 _target, float3 _up, float _fovy, int _w, int _h):
position(_pos), target(_target), up(_up), fovy(_fovy), width(_w), height(_h) 
{
    up = normalize(up);
    line_of_sight = target - position;

    w = -1 * line_of_sight;
    w = normalize(w);

    u = cross(up, w);
    u = normalize(u);

    v = cross(w, u);
    v = normalize(v);

    focalWidth = 1.0;
    aspect = (float)width / (float)height;
    focalHeight = focalWidth / aspect;

    focalDistance = focalHeight / (2.0 * tan(fovy * M_PI/(180.0 * 2.0)));
}

__device__ __host__ const float3 Camera::get_ray_direction(const int i, const int j) const
{
    float3 dir = make_float3(0.0, 0.0, 0.0);

    dir += (-1 * w) * focalDistance;
    float xw = aspect * (i - (width / 2.0) + 0.5) / width;
    float yw = (j - (height/2.0) + 0.5) / height;
    dir += (u * xw);
    dir += (v * yw);

    dir = normalize(dir);
    return dir;
}
