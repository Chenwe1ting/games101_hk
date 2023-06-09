//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box(两个点来确定一个包围盒左下 右上)
    Bounds3()//构造函数
    {
        double minNum = std::numeric_limits<double>::lowest();//最小值 而min是最小正值
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }//Diagonal 意为对角

    //三种情形的判断 包围盒的x y z哪一个值最大分别返回0 1 2
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    //包围盒的表面积
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    //包围盒的中心
    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    //得到两个包围盒的合并包围盒(更大的包围盒)
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }


    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }


    //判断是否有重叠部分
    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    //判断光线是否在包围盒内
    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    //重载了运算符[] 该函数的调用为内联函数的调用
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    //计算各个方向进入和出去包围盒的时间，然后得到总体的进入和出去包围盒的时间
    float x_mint = (pMin.x - ray.origin.x) * invDir[0];
    float x_maxt = (pMax.x - ray.origin.x) * invDir[0];
    float y_mint = (pMin.y - ray.origin.y) * invDir[1];
    float y_maxt = (pMax.y - ray.origin.y) * invDir[1];    
    float z_mint = (pMin.z - ray.origin.z) * invDir[2];
    float z_maxt = (pMax.z - ray.origin.z) * invDir[2];
    //若方向为负方向 则考虑交换tmin和tmax
    if (!dirIsNeg[0])
    {
        float t = x_mint;
        x_mint = x_maxt;
        x_maxt = t;
    }
    
    if (!dirIsNeg[1])
    {
        float t = y_mint;
        y_mint = y_maxt;
        y_maxt = t;
    }

    if (!dirIsNeg[2])
    {
        float t = z_mint;
        z_mint = z_maxt;
        z_maxt = t;
    }

    float t_enter = std::max(std::max(x_mint, y_mint), z_mint);
    float t_exit = std::min(std::min(x_maxt, y_maxt), z_maxt);
    if (t_enter < t_exit && t_exit>=0)
    {
        return true;
    }
    else
        return false;

}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
