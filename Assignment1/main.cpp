#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1,-eye_pos[2],
        0, 0, 0, 1;//平移矩阵

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotate;//角度制化为弧度制
    rotate << cos(rotation_angle*MY_PI/180), -sin(rotation_angle * MY_PI / 180), 0, 0,
        sin(rotation_angle * MY_PI / 180), cos(rotation_angle * MY_PI / 180), 0, 0,
        0, 0, 1, 0, 
        0, 0, 0, 1;//绕z轴旋转矩阵
    model = rotate * model;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)//45, 1, 0.1, 50
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();


    //Eigen::Matrix4f M_p = Eigen::Matrix4f::Identity();//这个是投影转正交矩阵
    //M_p << zNear, 0, 0, 0,
    //    0, zNear, 0, 0,
    //    0, 0, zNear + zFar, (-1.0 * zNear * zFar),
    //    0, 0, 1, 0;
    ////[l,r]   [b,t]    [f,n]
    //float angle = eye_fov * MY_PI / 180;     //求角度
    //float t = tan(angle / 2) * -zNear;      //更具直角三角形性质求tb（高）
    //float b = -1.0 * t;
    //float r = t * aspect_ratio;           //根据宽高比求（宽）
    //float l = -1.0 * r;

    //Eigen::Matrix4f M_s = Eigen::Matrix4f::Identity(); //这个是将立方体进行规范化（-1，1）
    //M_s << 2 / (r - l), 0, 0, 0,
    //    0, 2 / (t - b), 0, 0,
    //    0, 0, 2 / (zNear - zFar), 0,
    //    0, 0, 0, 1;

    //Eigen::Matrix4f M_t = Eigen::Matrix4f::Identity(); //这里是将三角形位移到原点
    //M_t << 1, 0, 0, (-1.0)* (r + l) / 2,
    //    0, 1, 0, (-1.0)* (t + b) / 2,
    //    0, 0, 1, (-1.0)* (zNear + zFar) / 2,
    //    0, 0, 0, 1;
    //projection = M_s * M_t * M_p * projection;   //这里是左乘所以是先进行透视转正交，然后位移，然后规范化

    projection << 
        (1 / tan(eye_fov/2* MY_PI / 180) / aspect_ratio), 0,0,0, 
        0, (1 / tan(eye_fov/2* MY_PI / 180)),0,0, 
        0, 0,-(zFar + zNear) / (zFar - zNear), (-2 * zFar * zNear) / (zFar - zNear), 
        0, 0, -1, 0;

        //(-1 / tan(eye_fov / 2 * MY_PI / 180) / aspect_ratio), 0, 0, 0,
        //0, (-1 / tan(eye_fov / 2 * MY_PI / 180)), 0, 0,
        //0, 0, -(zFar + zNear) / (zFar - zNear), (2 * zFar * zNear) / (zFar - zNear),
        //0, 0, 1, 0;
        // 
        // 
        //着重考虑zNear和zFar都是负的
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;//bool变量最好赋予初值
    std::string filename = "output.png";


    //通过自行传入参数来进行操作
    if (argc >= 3) {//接收到的参数大于三个，即检测到通过命令行传入参数时
        command_line = true;//设命令行开关标志为开
        angle = std::stof(argv[2]); //从命令行获取角度参数
        if (argc == 4) {//接收到的参数为四个，那么说明命令行输入了文件名参数
            filename = std::string(argv[3]);//从命令行获取文件名
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//若有一个坐标更小，则三角形不是等腰(近大远小，视觉)

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {//如果命令行开关标志为开（这一段if代码是为了应用命令行传入的参数，比如初始角度和文件名）

        r.clear(rst::Buffers::Color | rst::Buffers::Depth); //初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，所以不涉及深度，可以不管）


        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//设置、计算fram_buf的数据
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3,1.0f);
        
        //注意 CV_8U数据范围是{0-255} CV_32F数据范围是{0.0 - 1.0}


        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {//只要没有检测到按下ESC就循环(ESC的ASCII码是27)
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，所以不涉及深度，可以不管）


        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        //cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//若使用vec3b/vec3i则可以直接用CV_8UC3;但前面的格式是vec3f
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);//颜色范围此处为(0-255) BGR表示

        cv::imshow("image", image);
        key = cv::waitKey(10);

        cv::imwrite(filename, image);

        std::cout << "frame count: " << frame_count++ << '\n';//显示当前是第几帧画面

        if (key == 'A') {
            angle += 10;
        }
        else if (key == 'D') {
            angle -= 10;
        }
    }

    return 0;
}
