#include "base_detector.hpp"
#include "utile.hpp"

#ifndef ARROW_DETECTOR
#define ARROW_DETECTOR

namespace Engineering_robot_Pnx{

typedef std::vector<std::vector<cv::Point>> Counters;
typedef std::vector<cv::Point> Counter;
typedef std::vector<std::vector<cv::Point2d>> Counter2ds;
typedef std::vector<cv::Point2d> Counter2d;
typedef std::vector<std::vector<cv::Point2f>> Counter2fs;
typedef std::vector<cv::Point2f> Counter2f;


/**
 * @brief 用于存储一条线的信息
 * 
 * 这里的p1和p2代表的是角点的索引
 * 
 * 这里的slope代表的是角度
 * 
 */
struct Slope{
    int p1,p2;
    double slope;
};

class ArrowDetector : public BaseDetector{
public:
    /**
     * @brief 构造函数
     * @param config 配置文件
     * @param node 随便给个node，记得是要已经初始化完成的
     * @param name 检测器的名字
     */
    ArrowDetector(const YAML::Node& config, 
        rclcpp::Node::SharedPtr node, 
        const std::string & name);

    /**
     * @brief 构造函数 这个函数会根据 name 来自动生成 node
     * @param config 配置文件
     * @param name 检测器的名字
     */
    ArrowDetector(const YAML::Node& config, 
        const std::string & name);

    std::string const getDetectorName() override;

protected:
    /**
     * @brief 目标检测函数
     * @param input_data 输入数据
     * @param output_data 输出数据
     * @return 是否检测到对象
     * 
     * 注意错误处理
    */
    bool detect(InputData input_data,
        DetectorOutput& output_data) override;

    /**
     * @brief 图像预处理函数
     * @param pre_image 输入图像
     * @param binary_image 输出二值化图像
     * @param gray_image 输出灰度图像
     * 
     * 这个函数将会生成一个二值化的图像
     * 其首先会将蓝绿通道融合，然后在通过一些滤波和形态学操作
     * 
    */
    void imagePreprocess(const cv::Mat & pre_image, cv::Mat & binary_image, cv::Mat & gray_image);

    /**
     * @brief 角点检测的主函数
     * @param binary_image 输入图像
     * @param corners 返回的角点
     * @return 是否检测到目标
     * 
     * @TODO 写注释！！
     */
    bool targetArrow(const cv::Mat & binary_image, const cv::Mat & gray_image, Counter2f& corners);

    /**
     * @brief 寻找可能的轮廓
     * @brief 分成两步筛选，第一次是针对外围轮廓的初步筛选，第二次是针对平行线的筛选
     * @param binary_image 输入二值化图像
     * @param gray_image 输入的灰度图，主要用于subpixel角点检测
     * @param counters 返回的轮廓
     * @return 是否检测到目标
     * 
     * @todo 以后的方向，直接用机器学习判断是不是箭头
     */
    bool findCandidateContour(const cv::Mat& binary_image, 
        const cv::Mat& gray_image,
        Counter& counter);

    /**
     * @brief 从已经确定是六边形的轮廓中找出我们设定的8个点。并且会使用subpixel进行优化
     * @brief 
     * @param counter 输入的箭头轮廓
     * @param binary_image 输入的二值化图
     * @param gray_image 输入的灰度图
     * @param corners 输出的8个点，顺序为
     * 
     * @todo 注释
     */
    bool getCounterCorners(const Counter& counter, 
        const cv::Mat& binary_image, 
        const cv::Mat& gray_image, 
        Counter2f& corners);

    /**
     * @brief 是一个辅助函数，按照设定的方式对箭头的顶点进行排序
     * @param counter 输入的顶点，要求必须以顺时针或者逆时针排序
     * @param end_points 按照一定顺序排列的边
     * @todo 没写
     */
    void sortCorners(Counter& counter, std::vector<std::pair<cv::Point,cv::Point> > end_points);

// 参数部分与运行时全局变量
private:

    // 这是一个全局的变量，储存了在detect开始时会被绑定到output_data的image里面
    cv::Mat colored_image;

    // 加载参数
    bool loadConfig();

    // 二值化界限
    int BinaryThresholdThresh;
    // 二值化上界
    int BinaryThresholdMaxval;
    // findCandidateContour 中的多边形拟合参数Epsilon
    // 用于控制多边形拟合的精度
    double approxPolyDPEpsilon;
    // 第一次轮廓筛选，长边比短边的长度的最小比值
    double LongShortRateMin;
    // 第一次轮廓筛选，长边比短边的长度的最大比值
    double LongShortRateMax;
    // 第一次轮廓筛选，轮廓面积的最小值
    int PixelNumMin;
    // 第一次轮廓筛选，轮廓面积的最大值
    int PixelNumMax;
    // 第一次轮廓筛选，拟合矩形的最小长宽比
    double RectLengthWidthRatioMin;
    // 第一次轮廓筛选，拟合矩形的最大长宽比
    double RectLengthWidthRatioMax;
    // 第一次轮廓筛选，拟合多边形的最小边数
    int ApproxcurveSizeMin;
    // 第一次轮廓筛选，拟合多边形的最大边数
    int ApproxcurveSizeMax;

    // 第二次轮廓筛选，二分查找的平行线的初始左边界
    double InitHorizonLeftThreshold;
    // 第二次轮廓筛选，二分查找的平行线的初始右边界
    double InitHorizonRightThreshold;
    // canny函数的Threshold1
    double CannyThreshold1;
    // canny函数的Threshold2
    double CannyThreshold2;
    // canny函数的apertureSize
    double CannyapertureSize;
    // find_polygon_counter_points_sets 的顶点忽略范围
    double PeaksIgnoreRadius;

    // 解算3D点配置，8个点
    std::vector<cv::Point3f> object_points;
    // 兑换框正面四个点
    std::vector<cv::Point3f> redeem_front_points;
    // 相机内参
    std::vector<double> camera_matrix;
    // 相机曲变参数
    std::vector<double> dist_coeffs;

    static const double eps = 1e-8;

};

}// namespace Engineering_robot_Pnx

#endif