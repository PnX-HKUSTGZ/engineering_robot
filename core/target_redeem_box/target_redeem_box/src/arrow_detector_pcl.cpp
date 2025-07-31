#include "target_redeem_box/arrow_detector_pcl.hpp"

namespace Engineering_robot_Pnx{

std::string const ArrowDetectorPCL::getDetectorName(){
    return "ArrowDetectorPCL";
}

ArrowDetectorPCL::ArrowDetectorPCL(const YAML::Node& config, 
        rclcpp::Node::SharedPtr node, 
        const std::string & name)
    :ArrowDetector(config, node, name+"_attach_ArrowDetector"){
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] initing %s",name.c_str());
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] loading config %s",ArrowPath.c_str());
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
            throw std::runtime_error("ArrowDetectorPCL load config error");
        }
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] init %s successfully",name.c_str());
    }

ArrowDetectorPCL::ArrowDetectorPCL(const YAML::Node& config, 
    const std::string & name):
    ArrowDetector(config, name+"_attach_ArrowDetector"){
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] initing %s",name.c_str());
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] loading config %s",ArrowPath.c_str());
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
            throw std::runtime_error("ArrowDetectorPCL load config error");
        }
        RCLCPP_INFO(node_->get_logger(),"[ArrowDetectorPCL] init %s successfully",name.c_str());
    }

ArrowDetectorPCL::~ArrowDetectorPCL(){
}

bool ArrowDetectorPCL::loadConfig(){
    try {
        ArrowPath = config["ArrowPath"].as<std::string>();

        if (config["ArrowKDSearchRadius"]) {
            ArrowKDSearchRadius = config["ArrowKDSearchRadius"].as<double>();
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Config error: 'ArrowKDSearchRadius' is not defined in YAML!");
            return false;
        }

        if (config["ArrowViewPoint"] && config["ArrowViewPoint"].IsSequence()) {
            ArrowViewPoint = config["ArrowViewPoint"].as<std::vector<double>>();
            if (ArrowViewPoint.size() < 3) {
                RCLCPP_ERROR(node_->get_logger(), "Config error: 'ArrowViewPoint' must have at least 3 elements!");
                return false;
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Config error: 'ArrowViewPoint' is not defined or not a sequence in YAML!");
            return false;
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Load config from YAML failed: %s", e.what());
        return false;
    }

    try{
        arrow_point_cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile<pcl::PointXYZ>(ArrowPath, *arrow_point_cloud);

        // 计算目标点云的法向量
        pcl::PointCloud<pcl::Normal>::Ptr arrow_normals(new pcl::PointCloud<pcl::Normal>);
        // 法向量计算器
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        // kdtree 近邻搜索树
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        ne.setInputCloud(arrow_point_cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(ArrowKDSearchRadius);
        ne.setViewPoint(ArrowViewPoint[0],ArrowViewPoint[1],ArrowViewPoint[2]);
        ne.compute(*arrow_normals);

        arrow_point_cloud_normal = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::concatenateFields(*arrow_point_cloud, *arrow_normals, *arrow_point_cloud_normal);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(node_->get_logger(), " load arrow point cloud and compute normal vec error");
        return false;
    }

    return true;

}

bool ArrowDetectorPCL::detect(InputData input_data,
        DetectorOutput& output_data){

    if(!input_data.point_cloud_){
        RCLCPP_ERROR(node_->get_logger(), "[detect] input point cloud is empty!");
        return false;
    }
    if(!input_data.image){
        RCLCPP_ERROR(node_->get_logger(), "[detect] input point cloud is empty!");
        return false;
    }

    output_data.result_image_=std::make_shared<cv::Mat>(input_data.image->clone());

    colored_image = *output_data.result_image_;

    // 8个角点
    Counter2f corners;
    // pnp旋转向量
    cv::Mat pnp_rvec;
    // pnp平移向量
    cv::Mat pnp_tvec;

    if(!imageArrowDetect(pnp_rvec,pnp_tvec,corners)){
        RCLCPP_WARN(node_->get_logger(), "[detect] imageArrowDetect fail!");
        return false;
    }


    //点云的ROI
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    getROI(input_data.point_cloud_,roi_cloud,corners);

    // 发布 点云的ROI
    

    // ICP Point_to_plane

    output_data.point_cloud_=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cv::Mat icp_tvec;
    cv::Mat icp_rvec;
    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> aligned_point(new pcl::PointCloud<pcl::PointNormal>());

    output_data.point_cloud_=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::copyPointCloud(*aligned_point,*output_data.point_cloud_);

    if(!detectICP(arrow_point_cloud_normal,roi_cloud,icp_tvec,icp_rvec,aligned_point)){
        RCLCPP_WARN(node_->get_logger(), "[detect] detectICP fail!");
        return false;
    }

    return true;

}

bool ArrowDetectorPCL::detectICP(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & target_point_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & source_point_cloud,
    cv::Mat & tvec,
    cv::Mat & rvec,
    pcl::PointCloud<pcl::PointNormal>::Ptr & aligned_cloud){

    // 计算原点云的法向量
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    // 法向量计算器
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // kdtree 近邻搜索树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(source_point_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(SourceKDSearchRadius);
    ne.setViewPoint(0.0, 0.0, 0.0);
    ne.compute(*source_normals);

    // 目标点云和法向量合并
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*source_point_cloud, *source_normals, *cloud_source_with_normals);

    std::vector<int> indices_nan;
    pcl::removeNaNNormalsFromPointCloud(*cloud_source_with_normals, *cloud_source_with_normals, indices_nan);
    if (indices_nan.size() > 0) {
        RCLCPP_INFO(node_->get_logger(), "[detectICP] Target cloud with normals has %d NaN values.\n", (int)indices_nan.size());
    }
    if (cloud_source_with_normals->empty()){
        RCLCPP_ERROR(node_->get_logger(), "[detectICP] Target cloud with normals is empty after NaN removal.\n");
        return 0;
    }

    // 迭代最近点配准
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

    icp.setInputSource(cloud_source_with_normals);
    icp.setInputTarget(target_point_cloud);

    icp.setMaxCorrespondenceDistance(ICPMaxCorrespondenceDistance);
    icp.setMaximumIterations(MaximumIterations);
    icp.setTransformationEpsilon(TransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);

    // 结果点云
    icp.align(*aligned_cloud);

    if (!icp.hasConverged()) {
        RCLCPP_ERROR(node_->get_logger(), "[detectICP] ICP has not converged!");
        return false;
    }

    Eigen::Matrix4f final_transformation = icp.getFinalTransformation();

    // 从变换矩阵中提取旋转和平移
    Eigen::Matrix3f rotation_matrix = final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation_vector = final_transformation.block<3, 1>(0, 3);

    // 转换为旋转向量和平移向量
    cv::Mat rotation_matrix_cv = (cv::Mat_<double>(3, 3) <<
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

    cv::Rodrigues(rotation_matrix_cv, rvec);
    tvec = (cv::Mat_<double>(3, 1) << translation_vector(0), translation_vector(1), translation_vector(2));

    return true;

}

void ArrowDetectorPCL::getROI(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_pointcloud, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & output_pointcloud,
        const Counter2f & corners){

    if(corners.size()!=8){
        RCLCPP_ERROR(node_->get_logger(), "[getROI] corners size error!");
        throw std::runtime_error("ArrowDetectorPCL getROI corners size error!");
        return;
    }

    // 从corners中纠正顺序后的轮廓
    Counter2f counter;

    // 角点顺序
    static const std::vector<int> index={0,2,3,1,5,4};

    // 从corners中纠正顺序
    for(int i=0;i<6;i++){
        counter.push_back(corners[index[i]]);
    }
            
    for(const auto& point:input_pointcloud->points){
        Eigen::Matrix<double,3,1> point_eigen;
        Eigen::Matrix<double,3,1> image_point;
        point_eigen<<point.x,point.y,point.z;

        image_point=camera_matrix_eigen*point_eigen;
        image_point/=image_point(2);

        double dist=cv::pointPolygonTest(counter, cv::Point2f(image_point(0),image_point(1)),true);
        if(dist>=0){
            output_pointcloud->points.push_back(point);
        }
    }

}

bool ArrowDetectorPCL::imageArrowDetect(cv::Mat &rvec,
        cv::Mat &tvec,
        Counter2f &corners){

    cv::Mat binary_image;
    cv::Mat gray_image;

    try{
        imagePreprocess(colored_image,binary_image,gray_image);
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[imageArrowDetect] imagePreprocess fail! %s",e.what());
        return false;
    }

    cv::imshow("binary_image",binary_image);
    cv::imshow("gray_image",gray_image);
    cv::imshow("colored_image",colored_image);
    cv::waitKey(1);

    try{
        if(!targetArrow(binary_image,gray_image,corners)){
            RCLCPP_WARN(node_->get_logger(), "[imageArrowDetect] find corners fail!");
            return false;
        }
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[imageArrowDetect] find corners fail! %s",e.what());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[imageArrowDetect] find corners success!");

    try{
        bool pnp_solver_success=pnp_solver(corners,
            object_points,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            0,
            cv::SOLVEPNP_IPPE);
        
        if(!pnp_solver_success){
            RCLCPP_WARN(node_->get_logger(), "[imageArrowDetect] solve pnp fail!");
            return false;
        }
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[imageArrowDetect] solve pnp fail! %s",e.what());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[imageArrowDetect] solve pnp success!");

    return true;

}


} // Engineering_robot_Pnx