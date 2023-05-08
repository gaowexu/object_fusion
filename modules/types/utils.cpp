///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/types/utils.h"

namespace perception
{
namespace object_fusion
{

static constexpr const float epsilon = 1e-8F;

double ToSeconds(const Clock::time_point time)
{
    using Second = std::chrono::duration<double, std::ratio<1, 1>>;
    const auto seconds = std::chrono::duration_cast<Second>(time.time_since_epoch());
    return seconds.count();
}

double ToSeconds(const Clock::duration& duration)
{
    using Seconds = std::chrono::duration<double, std::ratio<1, 1>>;
    Seconds seconds{std::chrono::duration_cast<Seconds>(duration)};
    return seconds.count();
}

std::int64_t ToMilliseconds(const Clock::time_point time)
{
    std::chrono::milliseconds mill_seconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch());
    return mill_seconds.count();
}

void RotateAroundCenter(const Point& center, const float angle_cos, const float angle_sin, Point& p)
{
    const float new_x = (p.x - center.x) * angle_cos + (p.y - center.y) * (-angle_sin) + center.x;
    const float new_y = (p.x - center.x) * angle_sin + (p.y - center.y) * angle_cos + center.y;
    p.set(new_x, new_y);
}

float Cross(const Point& a, const Point& b)
{
    return a.x * b.y - a.y * b.x;
}

float Cross(const Point& p1, const Point& p2, const Point& p0)
{
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

bool PointComp(const Point& a, const Point& b, const Point& center)
{
    return atan2(a.y - center.y, a.x - center.x) > atan2(b.y - center.y, b.x - center.x);
}

bool CheckRectangeCross(const Point& p1, const Point& p2, const Point& q1, const Point& q2)
{
    bool ret = std::min(p1.x, p2.x) <= std::max(q1.x, q2.x) && std::min(q1.x, q2.x) <= std::max(p1.x, p2.x) &&
               std::min(p1.y, p2.y) <= std::max(q1.y, q2.y) && std::min(q1.y, q2.y) <= std::max(p1.y, p2.y);
    return ret;
}

bool CheckInsideBox2d(const float* box, const Point& p)
{
    const float margin = 1e-2F;
    float center_x = box[0], center_y = box[1];
    float angle_cos = static_cast<float>(cos(-box[6])), angle_sin = static_cast<float>(sin(-box[6]));
    float rot_x = (p.x - center_x) * angle_cos + (p.y - center_y) * (-angle_sin);
    float rot_y = (p.x - center_x) * angle_sin + (p.y - center_y) * angle_cos;

    return (fabs(rot_x) < box[3] / 2 + margin && fabs(rot_y) < box[4] / 2 + margin);
}

bool Intersection(const Point& p1, const Point& p0, const Point& q1, const Point& q0, Point& ans)
{
    if (CheckRectangeCross(p0, p1, q0, q1) == 0)
        return false;

    float s1 = Cross(q0, p1, p0);
    float s2 = Cross(p1, q1, p0);
    float s3 = Cross(p0, q1, q0);
    float s4 = Cross(q1, p1, q0);

    if (!(s1 * s2 > 0 && s3 * s4 > 0))
        return false;

    float s5 = Cross(q1, p1, p0);
    if (fabs(s5 - s1) > epsilon)
    {
        ans.x = (s5 * q0.x - s1 * q1.x) / (s5 - s1);
        ans.y = (s5 * q0.y - s1 * q1.y) / (s5 - s1);
    }
    else
    {
        float a0 = p0.y - p1.y, b0 = p1.x - p0.x, c0 = p0.x * p1.y - p1.x * p0.y;
        float a1 = q0.y - q1.y, b1 = q1.x - q0.x, c1 = q0.x * q1.y - q1.x * q0.y;
        float D = a0 * b1 - a1 * b0;

        ans.x = (b0 * c1 - b1 * c0) / D;
        ans.y = (a1 * c0 - a0 * c1) / D;
    }

    return true;
}

float BoxOverlap(const float* box_a, const float* box_b)
{
    float a_angle = box_a[6], b_angle = box_b[6];
    float a_dx_half = box_a[3] / 2, b_dx_half = box_b[3] / 2, a_dy_half = box_a[4] / 2, b_dy_half = box_b[4] / 2;
    float a_x1 = box_a[0] - a_dx_half, a_y1 = box_a[1] - a_dy_half;
    float a_x2 = box_a[0] + a_dx_half, a_y2 = box_a[1] + a_dy_half;
    float b_x1 = box_b[0] - b_dx_half, b_y1 = box_b[1] - b_dy_half;
    float b_x2 = box_b[0] + b_dx_half, b_y2 = box_b[1] + b_dy_half;

    Point center_a(box_a[0], box_a[1]);
    Point center_b(box_b[0], box_b[1]);

    Point box_a_corners[5];
    box_a_corners[0].set(a_x1, a_y1);
    box_a_corners[1].set(a_x2, a_y1);
    box_a_corners[2].set(a_x2, a_y2);
    box_a_corners[3].set(a_x1, a_y2);

    Point box_b_corners[5];
    box_b_corners[0].set(b_x1, b_y1);
    box_b_corners[1].set(b_x2, b_y1);
    box_b_corners[2].set(b_x2, b_y2);
    box_b_corners[3].set(b_x1, b_y2);

    const float a_angle_cos = static_cast<float>(cos(a_angle)), a_angle_sin = static_cast<float>(sin(a_angle));
    const float b_angle_cos = static_cast<float>(cos(b_angle)), b_angle_sin = static_cast<float>(sin(b_angle));

    for (std::int32_t k = 0; k < 4; k++)
    {
        RotateAroundCenter(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
        RotateAroundCenter(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
    }

    box_a_corners[4] = box_a_corners[0];
    box_b_corners[4] = box_b_corners[0];

    Point cross_points[16];
    Point poly_center;
    std::int32_t cnt = 0, flag = 0;

    poly_center.set(0, 0);
    for (std::int32_t i = 0; i < 4; i++)
    {
        for (std::int32_t j = 0; j < 4; j++)
        {
            flag = Intersection(
                box_a_corners[i + 1], box_a_corners[i], box_b_corners[j + 1], box_b_corners[j], cross_points[cnt]);
            if (flag)
            {
                poly_center = poly_center + cross_points[cnt];
                cnt++;
            }
        }
    }

    for (std::int32_t k = 0; k < 4; k++)
    {
        if (CheckInsideBox2d(box_a, box_b_corners[k]))
        {
            poly_center = poly_center + box_b_corners[k];
            cross_points[cnt] = box_b_corners[k];
            cnt++;
        }
        if (CheckInsideBox2d(box_b, box_a_corners[k]))
        {
            poly_center = poly_center + box_a_corners[k];
            cross_points[cnt] = box_a_corners[k];
            cnt++;
        }
    }

    poly_center.x /= static_cast<float>(cnt);
    poly_center.y /= static_cast<float>(cnt);

    Point temp;
    for (std::int32_t j = 0; j < cnt - 1; j++)
    {
        for (std::int32_t i = 0; i < cnt - j - 1; i++)
        {
            if (PointComp(cross_points[i], cross_points[i + 1], poly_center))
            {
                temp = cross_points[i];
                cross_points[i] = cross_points[i + 1];
                cross_points[i + 1] = temp;
            }
        }
    }

    float area = 0.0F;
    for (std::int32_t k = 0; k < cnt - 1; k++)
    {
        area += Cross(cross_points[k] - cross_points[0], cross_points[k + 1] - cross_points[0]);
    }

    return static_cast<float>(fabs(area)) / 2.0F;
}

float BEVIoU(const float* box_a, const float* box_b)
{
    float sa = box_a[3] * box_a[4];
    float sb = box_b[3] * box_b[4];
    float s_overlap = BoxOverlap(box_a, box_b);
    return s_overlap / fmaxf(sa + sb - s_overlap, epsilon);
}

float CalculateIoU(const Box3D& a, const Box3D& b)
{
    const float box_a[7] = {a.x, a.y, a.z, a.length, a.width, a.height, a.orientation};
    const float box_b[7] = {b.x, b.y, b.z, b.length, b.width, b.height, b.orientation};

    return BEVIoU(box_a, box_b);
}

float CalculateEuclideanDistance(const Box3D& a, const Box3D& b, const bool enable_z, const bool enable_norm)
{
    if (enable_z)
    {
        if (enable_norm)
            return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
        else
            return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    }
    else
    {
        if (enable_norm)
            return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
        else
            return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
    }
}

}  // namespace object_fusion
}  // namespace perception
