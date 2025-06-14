#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

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

#endif