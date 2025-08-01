#include "base_detector.hpp"
#include "arrow_detector.hpp"

#ifndef ARROW_DETECTOR_PCL
#define ARROW_DETECTOR_PCL
namespace Engineering_robot_Pnx{


class ArrowDetectorPCL : public ArrowDetector{

public:
    /**
     * @brief 构造函数
     * @param config 配置文件
     * @param node 随便给个node，记得是要已经初始化完成的
     * @param name 检测器的名字
     */
    ArrowDetectorPCL(const YAML::Node& config, 
        rclcpp::Node::SharedPtr node, 
        const std::string & name);

    /**
     * @brief 构造函数 这个函数会根据 name 来自动生成 node
     * @param config 配置文件
     * @param name 检测器的名字
     */
    ArrowDetectorPCL(const YAML::Node& config, 
        const std::string & name);

    std::string const getDetectorName() override;

    virtual ~ArrowDetectorPCL();

protected:

    // 加载参数
    bool loadConfig();

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
     * @brief 这个函数负责跑完 ``ArrowDetector`` 类的 ``detect`` 流程，但是不进行方向反转的检测，以及colored_image的赋值
     * @param rvec 旋转向量
     * @param tvec 平移向量
     * @param corners 对应顺序的8个点
     * @return 角点检测是否成功
     * 
    */
    bool imageArrowDetect(cv::Mat &rvec,
        cv::Mat &tvec,
        Counter2f &corners);

    /**
     * @brief 将三维点云投影到画面后，取出落在轮廓中的点
     * @param input_pointcloud 输入的点云
     * @param output_pointcloud 输出的点云
     * @param corners 对应顺序的8个点，从imageArrowDetect中得到
     */
    void getROI(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_pointcloud, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & output_pointcloud,
        const Counter2f & corners);

    /**
     * @brief 使用icp的 point_to_cloud 的变种来进行点云匹配 辅助函数
     * @param target_point_cloud 目标点云
     * @param source_point_cloud 待匹配点云
     * @param tvec 平移矩阵
     * @param rvec 旋转矩阵
     * @param aligned_cloud 匹配的点云
     * @return 是否成功
     */
    bool detectICP(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & target_point_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & source_point_cloud,
        cv::Mat & tvec,
        cv::Mat & rvec,
        pcl::PointCloud<pcl::PointNormal>::Ptr & aligned_cloud);

// 参数部分
protected:

    // 箭头点云的路径 从 share 的路径下找
    std::string ArrowPath;

    // 原点云 kdtree搜索树的搜索半径
    double SourceKDSearchRadius;

    // setMaxCorrespondenceDistance 的参数
    double ICPMaxCorrespondenceDistance;
    // setMaximumIterations ICP循环次数 终止条件
    int MaximumIterations;
    // setTransformationEpsilon 参数 终止条件
    double TransformationEpsilon;
    // setEuclideanFitnessEpsilon 参数 终止条件
    double EuclideanFitnessEpsilon;

    // 箭头点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr arrow_point_cloud=nullptr;
    // 带有法向量信息的箭头点云
    pcl::PointCloud<pcl::PointNormal>::Ptr arrow_point_cloud_normal=nullptr;
    // 计算箭头点云的法向量的时候的视角 xyz
    std::vector<double> ArrowViewPoint;
    // 目标点云 kdtree搜索树的搜索半径
    double ArrowKDSearchRadius;
    
private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr arrow_pcl_pub_;

};

}// namespace Engineering_robot_Pnx

#endif