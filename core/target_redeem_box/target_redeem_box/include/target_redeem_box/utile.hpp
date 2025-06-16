#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp> 

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#ifndef TARGET_REDEEM_BOX_UTILE
#define TARGET_REDEEM_BOX_UTILE

/**
 * @brief 计算两个点之间的距离
 * @param p1 第一个点
 * @param p2 第二个点
 * @return double 距离
 */
template<typename T,typename G>
double distance_points(const cv::Point_<T> & p1,const cv::Point_<G> & p2);

/**
 * @brief 计算一条直线相对于水平线的角度
 * @param p1 第一个点
 * @param p2 第二个点
 * @return double 角度 范围[0,180]度
 */
double angle_according_to_horizon(cv::Point p1,cv::Point p2);

/**
 * @brief pnp求解器,参数定义和 `cv::solvePnP` 一样
 * @param image_points2D 图像点
 * @param object_points3D 物体点
 * @param camera_matrix 相机内参
 * @param dist_coeffs 相机畸变系数
 * @param rvec 旋转向量
 * @param tvec 平移向量
 * @param useExtrinsicGuess 是否使用外部猜测
 * @param flags pnp求解方法
 * @return pnp是否成功
 */
bool pnp_solver(const std::vector<cv::Point2f > & image_points2D,
    const std::vector<cv::Point3f > & object_points3D,
    const std::vector<double> & camera_matrix,
    const std::vector<double> & dist_coeffs,
    cv::Mat & rvec, 
    cv::Mat & tvec, 
    bool useExtrinsicGuess, 
    int flags);

/**
 * @brief 绘制pnp结果，这个函数将会画出object_points中点依次连起来的轮廓，以及物体坐标系下的xyz轴
 * @tparam T 点的类型
 * @param image 输入图像
 * @param rvec 旋转向量
 * @param tvec 平移向量
 * @param camera_matrix 相机内参
 * @param object_points 物体点
 * @param color 颜色
 * @param thickness 线宽
 * @param textpos pnp结果文本位置
 * @param draw_xyz 是否绘制xyz轴
 */
template<typename T>
void draw_pnp_result(cv::Mat image, 
    const cv::Mat & rvec,
    const cv::Mat & tvec,
    const std::vector<double> & camera_matrix,
    const std::vector<cv::Point3_<T>> object_points,
    cv::Scalar color, 
    int thickness, 
    cv::Point textpos,
    bool draw_xyz);

/**
 * @brief 函数会从binart_image中提取出多边形的每一条边的点集合
 * @param binary_image 输入的二值化图像，这个图片应当是 `uchar` 或者 `CV_8UC1` 类型的，并且其中只有一个多边形轮廓
 * @param peaks 输入的多边形的顶点
 * @param peaks_ignore_radius 输入的多边形的顶点忽略半径
 * @param point_sets 输出的多边形的每一条边的点集合
 * @param end_points 输出的多边形的每一条边的端点
 */
void find_polygon_counter_points_sets(const cv::Mat & binary_image,
    const std::vector<cv::Point> & peaks,
    const double peaks_ignore_radius,
    std::vector<std::vector<cv::Point>> & point_sets,
    std::vector<std::pair<cv::Point,cv::Point> >& end_points);


/**
 * @brief 计算两条线的交点
 * @brief 对于直线的定义 In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
 * @param line1 第一条直线
 * @param line2 第二条直线
 * @return cv::Point2f 交点
 * @brief 注意，如果有平行的情况会抛出错误
 */
cv::Point2f get_intersection(const cv::Vec4d & line1, const cv::Vec4d & line2);

/**
 * @brief 得到两个点之间的直线
 * @tparam T 点的类型
 * @param p1 第一个点
 * @param p2 第二个点
 * @return cv::Vec4d 直线 对于直线的定义 In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
 */
template<typename T>
cv::Vec4d get_line(const cv::Point_<T> & p1,const cv::Point_<T> & p2);

/**
 * @brief 旋转向量转换为四元数
 * @param rotation_vector_mat 旋转向量，1*3或者3*1的矩阵 并且是 CV_32F或者CV_64F
 * @return geometry_msgs::msg::Quaternion 四元数 ，失败会直接返回一个无效四元数
 */
geometry_msgs::msg::Quaternion rotation_vector_to_quaternion(const cv::Mat& rotation_vector_mat);

/**
 * @brief 四元数转换为旋转向量
 * @param quaternion_msg 四元数
 * @return cv::Mat 旋转向量，3*1的矩阵，类型为CV_64F。失败会返回一个空的cv::Mat。
 */
cv::Mat quaternion_to_rotation_vector(const geometry_msgs::msg::Quaternion& quaternion_msg);

/**
 * @brief 计算点到直线的距离
 * @tparam T 点的类型
 * @param point 点
 * @param line 直线
 * @return double 距离
 */
template <typename T>
double distance_point_line(const cv::Point_<T>& point, const cv::Vec4d& line);

/**
 * @brief 绘制一条直线，无限长
 * @param image 输入图像
 * @param line 直线 对于直线的定义 In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line. 
 * @param color 颜色
 * @param thickness 线宽
 */
void draw_line(cv::Mat& image,const cv::Vec4d& line,cv::Scalar color,int thickness);

/**
 * @brief 绘制一条直线，无限长
 * @param image 输入图像
 * @param lines 直线 对于直线的定义 In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line. 
 * @param color 颜色
 * @param thickness 线宽
 */
void draw_lines(cv::Mat& image,const std::vector<cv::Vec4d>& lines,cv::Scalar color,int thickness);

#endif