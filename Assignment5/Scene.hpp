#pragma once

#include <vector>
#include <memory>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"

class Scene
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    double fov = 90;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 5;//追踪的次数
    float epsilon = 0.00001;

    Scene(int w, int h) : width(w), height(h)
    {}

    //重载了Add函数 添加Object和Light
    void Add(std::unique_ptr<Object> object) { objects.push_back(std::move(object)); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    //获得物体和灯光的记录vector
    [[nodiscard]] const std::vector<std::unique_ptr<Object> >& get_objects() const { return objects; }
    [[nodiscard]] const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }

private:
    // creating the scene (adding objects and lights)
    //std::vector<std::unique_ptr<Object> > objects;
    //场景的成员变量是所有的Object和Light,但是是以Unique_ptr的形式(智能指针)
    std::vector<std::unique_ptr<Object> > objects;//vector里面的元素是Object类型的Unique指针
    std::vector<std::unique_ptr<Light> > lights;//vector里面的元素是Light类型的Unique指针
};