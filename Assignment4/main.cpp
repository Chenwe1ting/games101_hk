#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) //括号内的参数是固定的
{
    //if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) //回调函数，点击鼠标左键
    if(event==cv::EVENT_RBUTTONDOWN&&control_points.size()<4)//回调函数，点击鼠标右键
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

#pragma region Bernstein Method
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}
#pragma endregion

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // de Casteljau's algorithm
    if (control_points.size() == 1) 
    return control_points[0];

    std::vector<cv::Point2f> temp;
    for (int i = 0; i < control_points.size()-1; i++)
    {
        temp.push_back((1 - t) * control_points[i] + t * control_points[i + 1]);
    }
    return recursive_bezier(temp,t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto p = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(p.y, p.x)[2] = 255;//(p.y,p.x)相反！！代表坐标的位置，[]内的0 1 2代表RGB各个通道的值
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));//Scalar（B，G，R）不同的值设置window的背景颜色
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) //ESC：ASCⅡ码27
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);//第一个3为圆的半径，第二个3为组成圆的线条的粗细程度
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
               bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
