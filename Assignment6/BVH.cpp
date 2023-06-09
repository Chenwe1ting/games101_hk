#include <algorithm>
#include <cassert>
#include "BVH.hpp"

//不是KD_tree等划分空间的加速结构，而是划分物体的加速结构
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
    primitives(std::move(p))
{
    time_t start, stop1,stop2;
    time(&start);
    if (primitives.empty())
        return;

    switch (splitMethod)
    {
    case SplitMethod::NAIVE:
        root = recursiveBuildBVH(primitives);//递归构建BVH树结构
            //计算时间
        time(&stop1);
        {double diff = difftime(stop1, start);
        int hrs = (int)diff / 3600;
        int mins = ((int)diff / 60) - (hrs * 60);
        int secs = (int)diff - (hrs * 3600) - (mins * 60);


        printf(
            "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
            hrs, mins, secs);//输出生成BVH结构的耗时
        }
        break;

    case SplitMethod::SAH:
        root = recursiveBuildSAH(primitives);
        //计算时间
        time(&stop2);
        {
            double diff2 = difftime(stop2, start);
            int hrs2 = (int)diff2 / 3600;
            int mins2 = ((int)diff2 / 60) - (hrs2 * 60);
            int secs2 = (int)diff2 - (hrs2 * 3600) - (mins2 * 60);

            printf(
                "\rSAH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
                hrs2, mins2, secs2);//输出生成BVH结构的耗时
        }
        break;
    
}

}


//分三类情形，一个物体 两个物体 和大于等于三个物体的情形 
BVHBuildNode* BVHAccel::recursiveBuildBVH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    //计算BVH一个节点中的所有primitives的包围盒
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());//三种类型的物体来获得其包围盒，并扩大包围盒
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildBVH(std::vector{objects[0]});
        node->right = recursiveBuildBVH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();//判断包围盒的哪一个坐标值是最大的(0 1 2分别对应 x y z)
        //x y z哪个对应的长度最长就按哪个来从小到大排序
        switch (dim) {
        case 0:
            //排序 并给定排序的规则，利用lambda表达式给定仿函数
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        //进行划分
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));//若不相等会弹出error

        node->left = recursiveBuildBVH(leftshapes);
        node->right = recursiveBuildBVH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode;

    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());//三种类型的物体来获得其包围盒，并扩大包围盒
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildBVH(std::vector{ objects[0] });
        node->right = recursiveBuildBVH(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else 
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();

        float SN = centroidBounds.SurfaceArea();
        int B = 10;
        int mincostIndex = 0;
        float mincost = std::numeric_limits<float>::infinity();//最小的花销

        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z; });
            break;
        }
        for (int i = 0; i < B; i++)
        {
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * i / B);
            auto ending = objects.end();
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            Bounds3 leftbound, rightbound;
            for (int k = 0; k < leftshapes.size(); k++)
            {
                leftbound = Union(leftbound, leftshapes[k]->getBounds().Centroid());
            }
            for (int k = 0; k < rightshapes.size(); k++)
            {
                rightbound = Union(rightbound, rightshapes[k]->getBounds().Centroid());
            }
            float SA = leftbound.SurfaceArea();
            float SB = rightbound.SurfaceArea();
            float tempcost = 0.125 + (leftshapes.size() * SA / SN) + (rightshapes.size() * SB / SN);
            if (tempcost < mincost)
            {
                mincost = tempcost;
                mincostIndex = i;
            }
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + objects.size() * (mincostIndex / B);
        auto ending = objects.end();
        
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        //检查是否元素仍然相等
        assert(leftshapes.size() + rightshapes.size() == objects.size());

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);


    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}


//递归求光线与逐层的BVH的交
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = (ray.direction[0] > 0);
    dirIsNeg[1] = (ray.direction[1] > 0);
    dirIsNeg[2] = (ray.direction[2] > 0);

    Intersection Inte;

    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return Inte;
    }

    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);
    }

    Intersection l = getIntersection(node->left,ray);
    Intersection r = getIntersection(node->right, ray);

    return l.distance < r.distance ? l : r;

    //return Inte;
}