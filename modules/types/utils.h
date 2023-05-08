///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_TYPES_UTILS_H
#define MODULES_TYPES_UTILS_H

#include <cmath>

#include "modules/time/clock.h"

namespace perception
{
namespace object_fusion
{

double ToSeconds(const Clock::time_point time);
double ToSeconds(const Clock::duration& duration);
std::int64_t ToMilliseconds(const Clock::time_point time);

struct Point
{
    float x, y;
    Point() {}
    Point(float _x, float _y) { x = static_cast<float>(_x), y = static_cast<float>(_y); }
    void set(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
    Point operator+(const Point& b) const { return Point(x + b.x, y + b.y); }
    Point operator-(const Point& b) const { return Point(x - b.x, y - b.y); }
};

void RotateAroundCenter(const Point& center, const float angle_cos, const float angle_sin, Point& p);

float Cross(const Point& a, const Point& b);

float Cross(const Point& p1, const Point& p2, const Point& p0);

bool PointComp(const Point& a, const Point& b, const Point& center);

bool CheckRectangeCross(const Point& p1, const Point& p2, const Point& q1, const Point& q2);

bool CheckInsideBox2d(const float* box, const Point& p);

bool Intersection(const Point& p1, const Point& p0, const Point& q1, const Point& q0, Point& ans);

float BoxOverlap(const float* box_a, const float* box_b);

float BEVIoU(const float* box_a, const float* box_b);

struct Box3D
{
    float x;
    float y;
    float z;
    float length;
    float width;
    float height;
    float orientation;

    Box3D(const float x,
          const float y,
          const float z,
          const float length,
          const float width,
          const float height,
          const float orientation)
        : x(x), y(y), z(z), length(length), width(width), height(height), orientation(orientation)
    {
    }
};

float CalculateIoU(const Box3D& a, const Box3D& b);

float CalculateEuclideanDistance(const Box3D& a,
                                 const Box3D& b,
                                 const bool enable_z = false,
                                 const bool enable_norm = true);

/// \brief Remaps angle to the interval [-pi, pi) using a modulus operation.
///
/// Formula: angle - 2 * pi * floor(angle / (2 * pi) + 0.5)
///
/// \tparam T Floating point type.
/// \param angle Value to wrap.
/// \return Radians in the interval [-pi, pi)
float wrap_to_pi(const float angle)
{
    const float two_pi = 2.0F * 3.14159F;
    float multiple = angle / two_pi;
    multiple = std::floor(multiple + 0.50F);
    return angle - two_pi * multiple;
}

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_TYPES_UTILS_H
