#pragma once

#include "rtweekend.h"

#include "color.h"
#include "hittable.h"
#include "material.h"
#include <iostream>

class camera
{
public:
    double aspect_ratio = 1.0;  // ͼ������߶�֮��
    int    image_width = 100;  // ������Ϊ��λ����Ⱦͼ����
    int    samples_per_pixel = 10;   // ÿ�����ص����������
    int    max_depth = 10;   // ���볡����������߷�������

    double vfov = 90;              // ��ֱ�ӽ�(�ӳ�)
    point3 lookfrom = point3(0, 0, -1);  // ����������⿪ʼ��
    point3 lookat = point3(0, 0, 0);   // ���������������
    vec3   vup = vec3(0, 1, 0);     // ������ "���� "�ķ���

    double defocus_angle = 0;  // ͨ��ÿ�����صĹ��ߵı仯�Ƕ�
    double focus_dist = 10;    // ��������۲�㵽��ȫ�۽�ƽ��ľ���

    void render(const hittable& world)
    {
        initialize();
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
        for (int j = 0; j < image_height; ++j)
        {
            std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
            for (int i = 0; i < image_width; ++i)
            {
                color pixel_color(0, 0, 0);
                for (int sample = 0; sample < samples_per_pixel; ++sample)
                {
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world);
                }
                writeColor(std::cout, pixel_color, samples_per_pixel);
            }
        }
    }
private:
    int    image_height;    // ��Ⱦͼ��ĸ߶�
    point3 camera_center;          // ���������
    point3 pixel00_loc;     // ���� 0, 0 ��λ��
    vec3   pixel_delta_u;   // �������ҵ�ƫ����
    vec3   pixel_delta_v;   // ���·����ص�ƫ����
    vec3   u, v, w;         // �����֡������
    vec3   defocus_disk_u;  // �۽���ˮƽ�뾶
    vec3   defocus_disk_v;  // �۽��̴�ֱ�뾶

    void initialize()
    {
        image_height = static_cast<int>(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        camera_center = lookfrom;

        // ȷ���ӿڳߴ�.
        auto theta = degrees_to_radians(vfov);
        auto h = tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (static_cast<double>(image_width) / image_height);

        // �������������ϵ�� u��v��w ��λ������
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // ����ˮƽ����ʹ�ֱ�ӿڱ�Ե��������
        vec3 viewport_u = viewport_width * u;    // �ӿ�ˮƽ��Ե������
        vec3 viewport_v = viewport_height * -v;  // �ӿڴ�ֱ��Ե������

        // �������ؼ��ˮƽ�ʹ�ֱ��������
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // �������Ͻ����ص�λ�á�
        auto viewport_upper_left = camera_center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        auto defocus_radius = focus_dist * tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    color ray_color(const ray& r, int depth, const hittable& world) const
    {
        hit_record rec;
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return color(0, 0, 0);
        if (world.hit(r, interval(0.001, infinity), rec))
        {
            ray scattered;
            color attenuation;
            if (rec.mat->scatter(r, rec, attenuation, scattered))
                return attenuation * ray_color(scattered, depth - 1, world);
            return color(0, 0, 0);
        }
        vec3 unit_direction = unit_vector(r.direction());
        auto a = 0.5 * (unit_direction.y() + 1.0);
        return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
    }

    ray get_ray(int i, int j) const
    {
        // Ϊλ�� i,j �������ػ�ȡ�����������������ߣ��ù���Դ�������ɢ����
        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = (defocus_angle <= 0) ? camera_center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    point3 defocus_disk_sample() const
    {
        // ����������뽹���е�һ�������
        auto p = random_in_unit_disk();
        return camera_center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    vec3 pixel_sample_square() const
    {
        // Returns a random point in the square surrounding a pixel at the origin.
        auto px = -0.5 + random_double();
        auto py = -0.5 + random_double();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }
};