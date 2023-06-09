//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        //坐标限定至[0,1]之间
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;  

        auto u_img = u * (width-1);
        auto v_img = (1-v)*(height-1);
        //因为uv坐标的范围是[0,1]，而像素坐标的范围是[0,width-1]和[0,height-1]。如果直接乘width,height。那么uv为1的时候，坐标就会越界。
        //但是作业框架使用的库对于越界的像素坐标并不会crash或报错，而是返回一个透明值，因此小牛uv为1的地方就没有颜色了。
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    //此处可以利用双线性插值来得到颜色值
    //Eigen::Vector3f getBilinear(float u, float v)
    //{
    //    float w1 = (int)u * (width - 1), h1 = (int)(1 - v) * (height - 1);
    //    float w2 = w1 + 1, h2 = h1 ;
    //    float w3 = w1, h3 = h1 + 1;
    //    float w4 = h1 + 1, h4 = h1 + 1;

    //    Eigen::Vector2f color1, color2, color3, color4, color5, color6, color;
    //    color1 = getColor(w1 / width, h1 / height);
    //    color2 = getColor(w2 / width, h2 / height);
    //    color3 = getColor(w3 / width, h3 / height);
    //    color4 = getColor(w4 / width, h4 / height);
    //    color5 = color1 + (color2 - color1) * (u * (width - 1) - w1);
    //    color6 = color3 + (color4 - color3) * (u * (width - 1) - w1);
    //    color = color5 + (color6 - color5) * ((1 - v) * (height - 1) - h1);
    //    return color;

    //}

};
#endif //RASTERIZER_TEXTURE_H
