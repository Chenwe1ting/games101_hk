// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)//判断点在三角形内外的算法
{
    Vector3f ve[3];
    ve[0] = Vector3f(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0);
    ve[1] = Vector3f(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0);
    ve[2] = Vector3f(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0);
    Vector3f res[3];
    for (int i = 0; i < 3; i++)
    {
        Vector3f d = Vector3f((float)x-_v[i].x(), (float)y-_v[i].y(), 0);
        res[i] = d.cross(ve[i]);
    }
    //如下仅仅判断z坐标的全部正负都相同也可以得到结果
    if ((res[0].z() > 0 && (res[1].z()&& res[2].z() > 0)) || (res[0].z() < 0 && (res[1].z() < 0)&&res[2].z()<0))
        return true;
    else
        return false;

    //Vector3f ve[3];
    //ve[0] = Vector3f(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0);
    //ve[1] = Vector3f(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0);
    //ve[2] = Vector3f(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0);
    //Vector3f d[3];
    //for (int i = 0; i < 3; i++)
    //{
    //    Vector3f temp(x - _v[i].x(), y - _v[i].y(), 0);
    //    d[i]=temp.cross(ve[i]);
    //}
    //if ((d[0].z() >=0 && d[1].z() >= 0 && d[2].z() >= 0) || (d[0].z() <= 0 && d[1].z() <= 0 && d[2].z() <= 0))
    //    return true;
    //else
    //    return false;


    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)//计算
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//遍历ind
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    auto vt = t.toVector4();//得到齐次坐标
    Vector2f AABB[2];//包围盒
    AABB[0] = Vector2f(std::min(std::min(vt[0].x(),vt[1].x()), vt[2].x()), std::min(std::min(vt[0].y(), vt[1].y()), vt[2].y()));
    AABB[1] = Vector2f(std::max(std::max(vt[0].x(), vt[1].x()), vt[2].x()), std::max(std::max(vt[0].y(), vt[1].y()), vt[2].y()));

    for (float x = AABB[0].x(); x <=AABB[1].x(); x++)
    {
        for (float y = AABB[0].y(); y <= AABB[1].y(); y++)
        {
            if (insideTriangle(x, y, t.v))
            {
                //计算重心坐标
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);

                //矫正透视插值(reciprocal意为倒数)
                //float w_reciprocal = 1/(alpha/vt[0].w()+beta/vt[1].w()+gamma/vt[2].w());//因为之前的齐次坐标的w值都设置为了1.0f
                //float z_interpolated = alpha * (vt[0].z() / vt[0].w()) + beta * (vt[1].z() / vt[1].w()) + gamma * (vt[2].z() / vt[2].w());
                //z_interpolated *= w_reciprocal;

                float z_interpolated = alpha * vt[0].z() + beta * vt[1].z() + gamma * vt[2].z();//不进行透视插值矫正的情况

                if (z_interpolated < depth_buf[get_index(x, y)])
                {
                    depth_buf[get_index(x,y)] = z_interpolated;//更新深度
                    Vector3f color = (alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2]);//t.color[i]为在i点处的颜色数据，数据类型为Vector3f
                    //将颜色写进颜色缓存
                    frame_buf[get_index(x, y)] = color * 255.0f;
                    
                }
            }
        }
    }



















    //auto v = t.toVector4();
    //Vector2f bbox[2];
    ////AABB包围盒
    //bbox[1] = Vector2f(std::max(std::max(v[0].x(), v[1].x()), v[2].x()), std::max(std::max(v[0].y(), v[1].y()), v[2].y()));
    //bbox[0] = Vector2f(std::min(std::min(v[0].x(), v[1].x()), v[2].x()), std::min(std::min(v[0].y(), v[1].y()), v[2].y()));
    //for (float x = bbox[0].x(); x <= bbox[1].x(); ++x)
    //{
    //    for (float y = bbox[0].y(); y <= bbox[1].y(); ++y)
    //    {
    //        if (insideTriangle(x, y, t.v))
    //        {
    //            //计算重心坐标
    //            float alpha, beta, gamma;
    //            std::tie(alpha,beta,gamma) = computeBarycentric2D(x, y, t.v);//解包获得元组的元素值，需要先声明变量，见上一行

    //            //透视矫正插值
    //            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //            z_interpolated *= w_reciprocal;
    //            //深度测试
    //            if (z_interpolated < depth_buf[get_index(x, y)])
    //            {
    //                //更新深度
    //                depth_buf[get_index(x, y)] = z_interpolated;
    //                Vector3f color = alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2];
    //                //将颜色写入颜色缓存
    //                frame_buf[get_index(x, y)] = color * 255;
    //            }
    //        }
    //    }
    //}

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on