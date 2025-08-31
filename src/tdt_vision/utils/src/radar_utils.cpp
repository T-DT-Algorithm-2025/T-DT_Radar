#include "radar_utils.h"

#define ARMOR_HEIGHT 0.15
namespace tdt_radar {
bool isPointInsideScreen(cv::Point2f point, int screenWidth,
                         int screenHeight)
{
    return point.x >= 0 && point.x <= screenWidth && point.y >= 0 &&
           point.y <= screenHeight;
}
parser::parser()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    std::cout << world_tvec << std::endl;
    std::cout << world_rvec << std::endl;
    fs.release();

    cv::FileStorage fs1;
    fs1.open("./config/camera_params.yaml", cv::FileStorage::READ);
    fs1["camera_matrix"] >> this->camera_matrix;
    fs1["dist_coeffs"] >> this->dist_coeffs;
    fs1.release();
    std::cout << "Resolve camera" << camera_matrix << std::endl;
    std::cout << "Resolve dist" << dist_coeffs << std::endl;

    points_map["Middle_Line"] = new Parser_Points("Middle_Line");
    points_map["Left_Road"] = new Parser_Points("Left_Road");
    points_map["Right_Road"] = new Parser_Points("Right_Road");
    points_map["Enemy_Buff"] = new Parser_Points("Enemy_Buff");
    points_map["Self_Fortress"] = new Parser_Points("Self_Fortress");
    points_map["Enemy_Fortress"] = new Parser_Points("Enemy_Fortress");

    points_map["Middle_Line"]->Height = 0.3;
    points_map["Left_Road"]->Height = 0.2;
    points_map["Right_Road"]->Height = 0.2;
    points_map["Enemy_Buff"]->Height = 0.6;
    points_map["Self_Fortress"]->Height = 0.15;
    points_map["Enemy_Fortress"]->Height = 0.15;
}
void parser::Change_Matrix()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();

    for (auto& points : points_map) {
        points.second->Update();
    }
}
void parser::draw_ui(cv::Mat& img)
{
    for (auto& points : points_map) {
        cv::polylines(img, points.second->Points_2D, true,
                      cv::Scalar(255, 255, 255));
    }
}
cv::Point2f parser::parse(cv::Point2f& input_point)
{
    float temp_height = get_height(input_point);
    if (temp_height > 0.79) {
        return cv::Point2f(19.322, -1.915);
    }
    return get_2d(input_point, temp_height);
}
float parser::get_height(cv::Point2f& input_point)
{
    for (auto& points : points_map) {
        if (points.second->return_height(input_point)) {
            return points.second->Height;
        }
    }
    return 0;
}
cv::Point2f parser::get_2d(cv::Point2f& input_point, float height)
{
    std::vector<cv::Point3f> world_points;
    world_points.push_back(cv::Point3f(12, -6, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16, -6, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16, -8, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(12, -8, ARMOR_HEIGHT + height));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, image_points);
    for (auto& point : image_points) {
    }
    std::vector<cv::Point2f> world_points2D;
    world_points2D.push_back(cv::Point2f(12, -6));
    world_points2D.push_back(cv::Point2f(16, -6));
    world_points2D.push_back(cv::Point2f(16, -8));
    world_points2D.push_back(cv::Point2f(12, -8));
    cv::Mat Perspective_matrix =
        cv::getPerspectiveTransform(image_points, world_points2D);
    cv::Mat srcPointMat(1, 1, CV_32FC2);
    srcPointMat.at<cv::Point2f>(0, 0) = input_point;
    cv::perspectiveTransform(srcPointMat, srcPointMat, Perspective_matrix);
    return srcPointMat.at<cv::Point2f>(0, 0);
}
std::vector<cv::Point3f>
Parser_Points::ReadPoints(const std::string& points_name)
{
    cv::FileStorage fs("./config/RM2025_Points.yaml",
                       cv::FileStorage::READ);  // 打开YAML文件

    if (!fs.isOpened()) {
        std::cout << "无法打开文件" << std::endl;
        exit(-1);
    }

    std::vector<cv::Point3f> points;

    cv::FileNode pointsNode = fs[points_name];
    if (pointsNode.type() != cv::FileNode::SEQ) {
        std::cout << "points节点不是序列" << std::endl;
        exit(-1);
    }

    for (auto&& it : pointsNode) {
        cv::Point3f point;
        it["x"] >> point.x;
        it["y"] >> point.y;
        it["z"] >> point.z;

        points.push_back(point);
    }
    return points;
}
std::vector<cv::Point>
Parser_Points::Float2Int(std::vector<cv::Point2f>& FloatPoint)
{
    std::vector<cv::Point> dstPoint;
    for (auto& i : FloatPoint) {
        dstPoint.emplace_back(int(i.x), int(i.y));
    }
    return dstPoint;
}
void Parser_Points::World2Camera()
{
    std::vector<cv::Point2f> temp_2D;
    cv::projectPoints(Points_3D, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, temp_2D);
    Points_2D = Float2Int(temp_2D);
}
Parser_Points::Parser_Points(const std::string& points_name)
{
    cv::FileStorage fs;
    fs.open("./config/camera_params.yaml", cv::FileStorage::READ);
    fs["camera_matrix"] >> this->camera_matrix;
    fs["dist_coeffs"] >> this->dist_coeffs;
    fs.release();

    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    std::vector<cv::Point3f> temp_3d = ReadPoints(points_name);
    this->Points_3D = temp_3d;
    World2Camera();
}

float Parser_Points::return_height(cv::Point2f& input_point)
{
    bool inside = false;
    if (cv::pointPolygonTest(
            Points_2D, cv::Point((int)input_point.x, (int)input_point.y),
            false) > 0) {
        return this->Height;
    } else {
        return 0;
    }
}
void Parser_Points::Update()
{
    cv::FileStorage fs;
    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    World2Camera();
}
}  // namespace tdt_radar