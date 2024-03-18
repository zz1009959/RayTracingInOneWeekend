#pragma once
#include"rtweekend.h"
class material;
class hit_record
{
public:
    point3 p;
    vec3 normal;
    shared_ptr<material> mat;
    double t;

    bool front_face;
    void setFaceNormal(const ray& r, const vec3& outward_normal)
    {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};
class hittable
{
public:
    //  虚析构函数:通常用于确保在继承体系中正确地释放资源
    virtual ~hittable() = default;
    //  纯虚函数，在基类中声明但没有提供实现，需要在派生类中实现
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};
