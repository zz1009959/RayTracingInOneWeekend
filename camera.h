#pragma once

#include "rtweekend.h"

#include "color.h"
#include "hittable.h"
#include "material.h"
#include <iostream>

class camera
{
public:
    double aspect_ratio = 1.0;  // 图像宽度与高度之比
    int    image_width = 100;  // 以像素为单位的渲染图像宽度
    int    samples_per_pixel = 10;   // 每个像素的随机样本数
    int    max_depth = 10;   // 进入场景的最大射线反弹次数

    double vfov = 90;              // 垂直视角(视场)
    point3 lookfrom = point3(0, 0, -1);  // 点摄像机从这开始看
    point3 lookat = point3(0, 0, 0);   // 点摄像机看向这里
    vec3   vup = vec3(0, 1, 0);     // 相机相对 "向上 "的方向

    double defocus_angle = 0;  // 通过每个像素的光线的变化角度
    double focus_dist = 10;    // 从摄像机观察点到完全聚焦平面的距离

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
    int    image_height;    // 渲染图像的高度
    point3 camera_center;          // 摄像机中心
    point3 pixel00_loc;     // 像素 0, 0 的位置
    vec3   pixel_delta_u;   // 像素向右的偏移量
    vec3   pixel_delta_v;   // 向下方像素的偏移量
    vec3   u, v, w;         // 摄像机帧基向量
    vec3   defocus_disk_u;  // 聚焦盘水平半径
    vec3   defocus_disk_v;  // 聚焦盘垂直半径

    void initialize()
    {
        image_height = static_cast<int>(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        camera_center = lookfrom;

        // 确定视口尺寸.
        auto theta = degrees_to_radians(vfov);
        auto h = tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (static_cast<double>(image_width) / image_height);

        // 计算摄像机坐标系的 u、v、w 单位基向量
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // 计算水平方向和垂直视口边缘的向量。
        vec3 viewport_u = viewport_width * u;    // 视口水平边缘的向量
        vec3 viewport_v = viewport_height * -v;  // 视口垂直边缘的向量

        // 计算像素间的水平和垂直增向量。
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // 计算左上角像素的位置。
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
        // 为位置 i,j 处的像素获取随机采样的摄像机光线，该光线源自摄像机散焦盘
        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = (defocus_angle <= 0) ? camera_center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    point3 defocus_disk_sample() const
    {
        // 返回摄像机离焦盘中的一个随机点
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