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
    //  ����������:ͨ������ȷ���ڼ̳���ϵ����ȷ���ͷ���Դ
    virtual ~hittable() = default;
    //  ���麯�����ڻ�����������û���ṩʵ�֣���Ҫ����������ʵ��
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};
