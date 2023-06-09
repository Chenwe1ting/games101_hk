#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
void test01()
{
    Scene scene(1280, 960);

    MeshTriangle bunny("../models/bunny/bunny.obj"); 
    //MeshTriangle eagle("../models/bunny/eagle.obj");
    //MeshTriangle cube1("../models/bunny/cubic_box.obj");
    //MeshTriangle cube2("../models/bunny/cubic_box2.obj");

    scene.Add(&bunny);
    //scene.Add(&eagle);
    //scene.Add(&cube1);
    //scene.Add(&cube2);
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));
    scene.buildBVH(BVHAccel::SplitMethod::NAIVE);


    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";//\n也代表换行
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    //记录渲染运行的时间

}
void test02()
{
    Scene scene(1280, 960);

    MeshTriangle bunny("../models/bunny/bunny.obj");
    //MeshTriangle eagle("../models/bunny/eagle.obj");
    //MeshTriangle cube1("../models/bunny/cubic_box.obj");
    //MeshTriangle cube2("../models/bunny/cubic_box2.obj");

    scene.Add(&bunny);
    //scene.Add(&eagle);
    //scene.Add(&cube1);
    //scene.Add(&cube2);
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));
    scene.buildBVH(BVHAccel::SplitMethod::SAH);


    Renderer r;

    auto start2 = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop2 = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop2 - start2).count() << " hours\n";//\n也代表换行
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop2 - start2).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop2 - start2).count() << " seconds\n";

    //记录渲染运行的时间

}

int main(int argc, char** argv)
{
    test01();
    test02();

    return 0;
}