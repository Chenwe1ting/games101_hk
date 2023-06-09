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
        0, 0, 0, 1;//ƽ�ƾ���

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotate;//�Ƕ��ƻ�Ϊ������
    rotate << cos(rotation_angle*MY_PI/180), -sin(rotation_angle * MY_PI / 180), 0, 0,
        sin(rotation_angle * MY_PI / 180), cos(rotation_angle * MY_PI / 180), 0, 0,
        0, 0, 1, 0, 
        0, 0, 0, 1;//��z����ת����
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


    //Eigen::Matrix4f M_p = Eigen::Matrix4f::Identity();//�����ͶӰת��������
    //M_p << zNear, 0, 0, 0,
    //    0, zNear, 0, 0,
    //    0, 0, zNear + zFar, (-1.0 * zNear * zFar),
    //    0, 0, 1, 0;
    ////[l,r]   [b,t]    [f,n]
    //float angle = eye_fov * MY_PI / 180;     //��Ƕ�
    //float t = tan(angle / 2) * -zNear;      //����ֱ��������������tb���ߣ�
    //float b = -1.0 * t;
    //float r = t * aspect_ratio;           //���ݿ��߱��󣨿���
    //float l = -1.0 * r;

    //Eigen::Matrix4f M_s = Eigen::Matrix4f::Identity(); //����ǽ���������й淶����-1��1��
    //M_s << 2 / (r - l), 0, 0, 0,
    //    0, 2 / (t - b), 0, 0,
    //    0, 0, 2 / (zNear - zFar), 0,
    //    0, 0, 0, 1;

    //Eigen::Matrix4f M_t = Eigen::Matrix4f::Identity(); //�����ǽ�������λ�Ƶ�ԭ��
    //M_t << 1, 0, 0, (-1.0)* (r + l) / 2,
    //    0, 1, 0, (-1.0)* (t + b) / 2,
    //    0, 0, 1, (-1.0)* (zNear + zFar) / 2,
    //    0, 0, 0, 1;
    //projection = M_s * M_t * M_p * projection;   //����������������Ƚ���͸��ת������Ȼ��λ�ƣ�Ȼ��淶��

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
        //���ؿ���zNear��zFar���Ǹ���
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;//bool������ø����ֵ
    std::string filename = "output.png";


    //ͨ�����д�����������в���
    if (argc >= 3) {//���յ��Ĳ�����������������⵽ͨ�������д������ʱ
        command_line = true;//�������п��ر�־Ϊ��
        angle = std::stof(argv[2]); //�������л�ȡ�ǶȲ���
        if (argc == 4) {//���յ��Ĳ���Ϊ�ĸ�����ô˵���������������ļ�������
            filename = std::string(argv[3]);//�������л�ȡ�ļ���
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//����һ�������С���������β��ǵ���(����ԶС���Ӿ�)

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {//��������п��ر�־Ϊ������һ��if������Ϊ��Ӧ�������д���Ĳ����������ʼ�ǶȺ��ļ�����

        r.clear(rst::Buffers::Color | rst::Buffers::Depth); //��ʼ��֡�������Ȼ��棨������ҵ������ҵֻ�漰һ��ͼ�Σ����Բ��漰��ȣ����Բ��ܣ�


        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//���á�����fram_buf������
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3,1.0f);
        
        //ע�� CV_8U���ݷ�Χ��{0-255} CV_32F���ݷ�Χ��{0.0 - 1.0}


        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {//ֻҪû�м�⵽����ESC��ѭ��(ESC��ASCII����27)
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//��ʼ��֡�������Ȼ��棨������ҵ������ҵֻ�漰һ��ͼ�Σ����Բ��漰��ȣ����Բ��ܣ�


        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        //cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//��ʹ��vec3b/vec3i�����ֱ����CV_8UC3;��ǰ��ĸ�ʽ��vec3f
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);//��ɫ��Χ�˴�Ϊ(0-255) BGR��ʾ

        cv::imshow("image", image);
        key = cv::waitKey(10);

        cv::imwrite(filename, image);

        std::cout << "frame count: " << frame_count++ << '\n';//��ʾ��ǰ�ǵڼ�֡����

        if (key == 'A') {
            angle += 10;
        }
        else if (key == 'D') {
            angle -= 10;
        }
    }

    return 0;
}