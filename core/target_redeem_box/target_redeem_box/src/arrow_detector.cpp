#include "target_redeem_box/arrow_detector.hpp"
#include "target_redeem_box/utile.hpp"
#include <omp.h>


namespace Engineering_robot_Pnx{

std::string const ArrowDetector::getDetectorName(){
    return "ArrowDetector";
}

ArrowDetector::ArrowDetector(const YAML::Node& config, 
        rclcpp::Node::SharedPtr node, 
        const std::string & name)
    :BaseDetector(config, node, name){
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
            throw std::runtime_error("ArrowDetector load config error");
        }
    }

ArrowDetector::ArrowDetector(const YAML::Node& config, 
    const std::string & name):
    BaseDetector(config, name){
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
            throw std::runtime_error("ArrowDetector load config error");
        }
    }

void ArrowDetector::imagePreprocess(const cv::Mat & pre_image, cv::Mat & pos_image, cv::Mat & gray_image){
    // 以BGR通道为基准 储存每个通道的图像
    std::vector<cv::Mat> SplitImage;
    cv::split(pre_image,SplitImage);

    cv::Mat GreyImage(SplitImage[0].size(),SplitImage[0].type());

    cv::addWeighted(SplitImage[0], 1, SplitImage[2], 1, 0, GreyImage);

    // 高斯滤波
    cv::Mat& GaussBinaryImage = gray_image;
    cv::GaussianBlur(GreyImage,
        GaussBinaryImage,
        cv::Size(5,5),
        1.5
    );

    // 锐化处理
    cv::Mat sharpening_kenel=(cv::Mat_<float>(3,3)<<
        0,-1,0,
        -1,5,-1,
        0,-1,0
    );
    cv::Mat Sharpened;
    cv::filter2D(GaussBinaryImage,Sharpened,-1,sharpening_kenel);

    cv::threshold(GaussBinaryImage,
        pos_image,
        BinaryThresholdThresh,
        BinaryThresholdMaxval,
        cv::THRESH_BINARY);

}

bool ArrowDetector::findCandidateContour(const cv::Mat& binary_image, const cv::Mat& gray_image, Counter& counter){

    (void)gray_image;

    // 从 binary_image 提取到的全部轮廓
    Counters all_counters;

    cv::findContours(binary_image,all_counters,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

    // 第一次筛选，粗略判断六边形时候成立

    // 第一次筛选后的轮廓
    Counters first_counters;
    // 第一次筛选后的轮廓,对应的拟合后的多边形
    Counters first_approxcurve_counters;
    // 第一次筛选后的轮廓，对应的轮廓像素大小
    std::vector<int> first_counters_size;
    // 筛选后的轮廓的互斥锁，用于之后的 omp 并行处理
    // std::mutex first_counters_mutex;

    // omp_set_num_threads(2);
    // #pragma omp parallel for
    for(auto & counter: all_counters){
        // 拟合最小矩形
        cv::RotatedRect rotatedrect_=cv::minAreaRect(counter);

        // 计算长宽比
        double LengthWidthRatio= (std::min(rotatedrect_.size.width,rotatedrect_.size.height)<=eps ? 
            -1 : std::max(rotatedrect_.size.width,rotatedrect_.size.height)/std::min(rotatedrect_.size.width,rotatedrect_.size.height));
        
        // 检查长宽比
        bool lwratio=(RectLengthWidthRatioMin<=LengthWidthRatio&&
            LengthWidthRatio<=RectLengthWidthRatioMax);
        if(!lwratio) continue;

        // 计算像素数量
        int pixel_num=cv::contourArea(counter);

        // 检查像素数量
        bool pixel_num_check=(PixelNumMin<=pixel_num&&
            pixel_num<=PixelNumMax);
        if(!pixel_num_check) continue;

        // 多边形拟合
        Counter approxcurve;
        cv::approxPolyDP(counter,approxcurve,approxPolyDPEpsilon,1);

        // 检查多边形边数
        bool approxsize=(std::size_t(ApproxcurveSizeMin)<=approxcurve.size()&&
            approxcurve.size()<=std::size_t(ApproxcurveSizeMax));
        if(!approxsize) continue;

        //边长长度筛选

        // 最长的四个边的平均值和其余边的平均值之比
        double approxcurve_length_rate=-1;

        if(approxcurve.size()==6){
            // 多边形边长储存
            std::vector<double> approxcurve_length;
            //前四长边长的平均值
            double average_length_4=0;
            //其余边长的平均值
            double average_rest_length=0;
            for(int siz_approxcurve=approxcurve.size(),i=approxcurve.size()-1;i>=0;i--){
                approxcurve_length.push_back(distance_points(approxcurve[i],approxcurve[(i+1)%siz_approxcurve]));
            }
            sort(approxcurve_length.begin(),approxcurve_length.end(),[](auto a,auto b){
                return a>b;
            });
            for(int i=0;i<4;i++) average_length_4+=approxcurve_length[i]/4;
            for(std::size_t i=4;i<approxcurve_length.size();i++) average_rest_length+=approxcurve_length[i]/(approxcurve_length.size()-3);
            approxcurve_length_rate=average_length_4/average_rest_length;
        }
        else{
            continue;
        }

        // 检查判断长边比短边的长度的比值
        bool approxcurve_length_rate_check=(
            LongShortRateMin<=approxcurve_length_rate&&
            approxcurve_length_rate<=LongShortRateMax
        );
        if(!approxcurve_length_rate_check) continue;

        // 添加备选
        {
            // std::lock_guard<std::mutex> lock(first_counters_mutex);
            first_counters.push_back(counter);
            first_approxcurve_counters.push_back(approxcurve);
            first_counters_size.push_back(pixel_num);
        }

    }

    if(first_counters.size()<=0){
        RCLCPP_INFO(node_->get_logger(), "[findCandidateContour] first filter fail!");
        return false;
    }

    // 第二次筛选，聚焦于平行线的筛选

    // 第二次筛选后的轮廓
    Counters second_counters;
    // 第二次筛选后的轮廓,对应的拟合后的多边形
    Counters second_approxcurve_counters;
    // 第二次筛选后的轮廓，对应的轮廓像素大小
    std::vector<int> second_counters_size;
    // 筛选后的轮廓的互斥锁，用于之后的 omp 并行处理
    // std::mutex second_counters_mutex;

    // omp_set_num_threads(2);

    // #pragma omp parallel for
    for(std::size_t i=0;i<first_counters.size();i++){
        // 储存每一个直线的点的索引以及角度
        std::vector<Slope> slopes;

        // 存下每个边的角度，并且对于在180度附近的角度将其认为是 0 
        for(int e=first_approxcurve_counters[i].size()-1,siz=first_approxcurve_counters[i].size();e>=0;e--){
            slopes.push_back(
                Slope{
                    e,
                    (e+1)%siz,
                    [](cv::Point p1,cv::Point p2){
                        double angle=angle_according_to_horizon(p1,p2);
                        return abs(angle-180)<5 ? 0 : angle;}(first_approxcurve_counters[i][e],first_approxcurve_counters[i][(e+1)%siz])
                    });
        }

        std::sort(slopes.begin(),slopes.end(),[](const Slope & a,const Slope & b){
            return a.slope<b.slope;
        });

        // 二分尝试次数
        int try_time=0;

        // 储存若干对平行的线
        std::vector<std::pair<Slope,Slope>> horizon_slope_pairs;
        // 储存已经使用过的线的标签
        std::vector<bool> used_line_tags(slopes.size(),0);

        // 平行线判断点，使用在二分的时候生成
        double HorizonThreshold=10;
        // 第二轮，二分查找的左边界
        double LThreshold=InitHorizonLeftThreshold;
        // 第二轮，二分查找的右边界
        double RThreshold=InitHorizonRightThreshold;


        while(horizon_slope_pairs.size()!=3&&try_time<=20){
            horizon_slope_pairs.clear();
            for(int i=slopes.size()-1;i>=0;i--) used_line_tags[i]=0;

            HorizonThreshold=(RThreshold+LThreshold)/2;
            for(int i=slopes.size()-1;i;i--){
                if(used_line_tags[i]||std::abs(slopes[i].slope-slopes[i-1].slope)>HorizonThreshold) continue;
                horizon_slope_pairs.push_back(std::make_pair(slopes[i],slopes[i-1]));
                used_line_tags[i]=used_line_tags[i-1]=1;
            }
            if(horizon_slope_pairs.size()==3) break;
            if(horizon_slope_pairs.size()<3) LThreshold=HorizonThreshold;
            else if(horizon_slope_pairs.size()>3) RThreshold=HorizonThreshold;
            try_time++;
        }
        if(horizon_slope_pairs.size()!=3){
            continue;
        }

        // 筛选后的轮廓添加
        {
            // std::lock_guard<std::mutex> lock(second_counters_mutex);
            second_counters.push_back(first_counters[i]);
            second_approxcurve_counters.push_back(first_approxcurve_counters[i]);
            second_counters_size.push_back(first_counters_size[i]);
        }

    }

    if(second_counters.size()<=0){
        RCLCPP_INFO(node_->get_logger(), "[findCandidateContour] second filter fail!");
        return false;
    }

    int max_pixel=0;
    int max_pixel_index=0;

    for(std::size_t i=0;i<second_counters_size.size();i++){
        if(second_counters_size[i]>max_pixel){
            max_pixel=second_counters_size[i];
            max_pixel_index=i;
        }
    }

    counter=second_counters[max_pixel_index];
    RCLCPP_INFO(node_->get_logger(), "[findCandidateContour] find %ld contours success!",second_counters.size());

    return true;
}

bool ArrowDetector::targetArrow(const cv::Mat & binary_image, const cv::Mat & gray_image, Counter2f& corners){
    Counter candidate_counter;
    try{
        if(!findCandidateContour(binary_image,gray_image,candidate_counter)){
            RCLCPP_WARN(node_->get_logger(), "[targetArrow] find candidate contours fail!");
            return false;
        }
    }
    catch(std::exception & e){
        RCLCPP_ERROR(node_->get_logger(), "[targetArrow] find candidate contours fail! with %s",e.what());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[targetArrow] find candidate contours success!");


    cv::drawContours(colored_image,Counters{candidate_counter},-1,cv::Scalar(200,32,255),1);

    // 直线拟合阶段

    try{
        if(!getCounterCorners(candidate_counter,binary_image,gray_image,corners)){
            RCLCPP_WARN(node_->get_logger(), "[targetArrow] get counter corners fail!");
            return false;
        }
    }
    catch(std::exception & e){
        RCLCPP_ERROR(node_->get_logger(), "[targetArrow] get counter corners fail! with %s",e.what());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[targetArrow] get counter corners success!");

    for(auto & corner : corners){
        cv::circle(colored_image,corner,1,cv::Scalar(123,32,176),-1);
    }

    return true;

}

bool ArrowDetector::getCounterCorners(const Counter& counter, 
    const cv::Mat& binary_image, 
    const cv::Mat& gray_image,
    Counter2f& corners){

    // 多边形拟合结果
    Counter approxcurve;
    // 按照规则排序后的每条边对应的顶点
    std::vector<std::pair<cv::Point,cv::Point> > sorted_end_points;

    try{
        cv::approxPolyDP(counter,approxcurve,approxPolyDPEpsilon,1);
    }
    catch(cv::Exception & e){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] approxPolyDP fail with %s",e.what());
        return false;
    }

    // 对 approxcurve 进行排序，确定每个点的位置

    sortCorners(approxcurve, sorted_end_points);
    if(sorted_end_points.size()!=6){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] size of sorted_end_points != 6, size: %ld",sorted_end_points.size());
        return false;
    }

    // 使用应用了 mask 的 binary_image
    cv::Mat masked_image;

    // 经过 canny 处理的 masked_image
    cv::Mat canny_image;

    {// circle mask

    // 轮廓的mask
    cv::Mat mask(binary_image.size(),CV_8UC1,cv::Scalar(0));
    
    // 对应轮廓的最小圆中心
    cv::Point2f center;
    // 对应轮廓的最小圆半径
    float radius;

    try{
        cv::minEnclosingCircle(approxcurve,center,radius);
    }
    catch(cv::Exception & e){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] minEnclosingCircle fail with %s",e.what());
        return false;
    }

    cv::circle(mask,cv::Point(center.x,center.y),radius,cv::Scalar(255),-1);

    cv::copyTo(binary_image, masked_image, mask);

    }

    try{
        cv::Canny(masked_image, canny_image, CannyThreshold1, CannyThreshold2, CannyapertureSize);
    }
    catch(cv::Exception & e){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] canny fail with %s",e.what());
        return false;
    }


    Counters point_sets;
    std::vector<std::pair<cv::Point,cv::Point> > end_points;

    find_polygon_counter_points_sets(canny_image, approxcurve, PeaksIgnoreRadius, point_sets, end_points);

    if(point_sets.size()!=6||end_points.size()!=6){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] size of point_sets or end_points != 6, point_sets: %ld end_points: %ld",point_sets.size(),end_points.size());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[getCounterCorners] find 6 points sets success!");

    // 符合sorted_end_points顺序的直线
    std::vector<cv::Vec4d> sorted_fitted_lines(6);

    // 拟合的直线，并且将其放在对应的位置
    for(int i=0;i<6;i++){
        cv::Vec4d line;
        cv::fitLine(point_sets[i],line,cv::DIST_L2,0,0.01,0.01);

        for(int e=0;e<6;e++){
            if((sorted_end_points[e].first==end_points[i].first && sorted_end_points[e].second==end_points[i].second) ||
                (sorted_end_points[e].second==end_points[i].first && sorted_end_points[e].first==end_points[i].second)){
                sorted_fitted_lines[e]=line;
                break;
            }
        }

    }

    // 画出拟合的直线
    draw_lines(colored_image,sorted_fitted_lines,cv::Scalar(255,255,255),1);
    for(int i=0;i<6;i++){
        cv::putText(colored_image,std::to_string(i),cv::Point(sorted_fitted_lines[i][2],sorted_fitted_lines[i][3]),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),2);
    }

    // 每个点对应的两条直线
    // 点的顺序是 见 readme.md
    const static std::vector<std::pair<int,int>> line_point_map={
        std::make_pair(0,1),
        std::make_pair(2,3),
        std::make_pair(0,4),
        std::make_pair(2,4),
        std::make_pair(1,5),
        std::make_pair(3,5),
        std::make_pair(0,3),
        std::make_pair(1,2)
    };

    // 每个直线对应的两个点，只是用前6个点
    // 顺序见 readme.md
    const static std::vector<std::pair<int,int>> point_line_map={
        std::make_pair(0,2),
        std::make_pair(0,4),
        std::make_pair(1,3),
        std::make_pair(1,5),
        std::make_pair(2,3),
        std::make_pair(4,5)
    };

    // 箭头的初始的6个角点
    Counter2f first_corners;

    for(int i=0;i<6;i++){
        first_corners.push_back(get_intersection(sorted_fitted_lines[line_point_map[i].first],sorted_fitted_lines[line_point_map[i].second]));
    }

    // 使用 SubPix 对 first_corners 进行优化

    // 优化过后的6个焦点角点
    Counter2f subpix_corners;
    // 优化后的6条直线
    std::vector<cv::Vec4f> subpix_lines;

    for(int i=0;i<6;i++){
        subpix_corners.push_back(cv::Point2f(first_corners[i].x,first_corners[i].y));
    }

    cv::TermCriteria cornerSubPix_criteria;
    cornerSubPix_criteria.maxCount=1000;
    cv::cornerSubPix(gray_image,subpix_corners,cv::Size(5,5),cv::Size(-1,-1),cornerSubPix_criteria);

    for(int i=0;i<6;i++){        
        subpix_lines.push_back(get_line(subpix_corners[point_line_map[i].first],subpix_corners[point_line_map[i].second]));
    }

    // 画出直线

    for(int i=0;i<6;i++){
        cv::line(colored_image,subpix_corners[point_line_map[i].first],subpix_corners[point_line_map[i].second],cv::Scalar(32,155,125),1);
    }
    
    for(int i=6;i<8;i++){
        subpix_corners.push_back(get_intersection(subpix_lines[line_point_map[i].first],subpix_lines[line_point_map[i].second]));
    }

    corners=subpix_corners;

    return true;

}

bool ArrowDetector::detect(InputData input_data, DetectorOutput& output_data){

    if(!input_data.image){
        RCLCPP_ERROR(node_->get_logger(), "[detect] image data is null !!");
        return 0;
    }

    output_data.result_image_=std::make_shared<cv::Mat>(input_data.image->clone());

    colored_image = *output_data.result_image_;

    cv::Mat binary_image;
    cv::Mat gray_image;

    try{
        imagePreprocess(colored_image,binary_image,gray_image);
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[detect] imagePreprocess fail! %s",e.what());
        return false;
    }

    cv::imshow("binary_image",binary_image);
    cv::imshow("gray_image",gray_image);
    cv::imshow("colored_image",colored_image);
    cv::waitKey(1);


    Counter2f corners;

    try{
        if(!targetArrow(binary_image,gray_image,corners)){
            RCLCPP_WARN(node_->get_logger(), "[detect] find corners fail!");
            return false;
        }
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[detect] find corners fail! %s",e.what());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[detect] find corners success!");

    cv::Mat rvec;
    cv::Mat tvec;

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
            RCLCPP_WARN(node_->get_logger(), "[detect] solve pnp fail!");
            return false;
        }
    }
    catch(cv::Exception& e){
        RCLCPP_ERROR(node_->get_logger(), "[detect] solve pnp fail! %s",e.what());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "[detect] solve pnp success!");


    {//判断反转
        cv::Point2f center;
        float radius;

        cv::minEnclosingCircle(corners,center,radius);

        if(center.x>corners[0].x){
            
            geometry_msgs::msg::TransformStamped stamp;
            stamp.transform.translation.x=tvec.at<double>(0);
            stamp.transform.translation.y=tvec.at<double>(1);
            stamp.transform.translation.z=tvec.at<double>(2);

            stamp.transform.rotation=rotation_vector_to_quaternion(rvec);

            // 反转
            stamp=reverseTransforme(stamp);

            // 转换回旋转向量
            rvec=quaternion_to_rotation_vector(stamp.transform.rotation);
            

            // 转换回平移向量
            tvec.at<double>(0)=stamp.transform.translation.x;
            tvec.at<double>(1)=stamp.transform.translation.y;
            tvec.at<double>(2)=stamp.transform.translation.z;
        }
    }

    // 画出结果

    for(int i=0;i<8;i++){
        cv::circle(colored_image,corners[i],1,cv::Scalar(123,32,176),-1);
        cv::putText(colored_image,std::to_string(i),corners[i],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(32,132,146),2);
    }

    draw_pnp_result(colored_image,rvec,tvec,camera_matrix,redeem_front_points,cv::Scalar(32,43,132),2,cv::Point(0,25),true);

    cv::imshow("result_image",colored_image);
    cv::waitKey(1);

    output_data.rvec=rvec;
    output_data.tvec=tvec;
    return true;
}

bool ArrowDetector::loadConfig(){
    try{
    BinaryThresholdThresh=config["BinaryThresholdThresh"].as<double>();
    BinaryThresholdMaxval=config["BinaryThresholdMaxval"].as<double>();
    approxPolyDPEpsilon=config["approxPolyDPEpsilon"].as<double>();
    LongShortRateMin=config["LongShortRateMin"].as<double>();
    LongShortRateMax=config["LongShortRateMax"].as<double>();
    PixelNumMin=config["PixelNumMin"].as<int>();
    PixelNumMax=config["PixelNumMax"].as<int>();
    RectLengthWidthRatioMin=config["RectLengthWidthRatioMin"].as<double>();
    RectLengthWidthRatioMax=config["RectLengthWidthRatioMax"].as<double>();
    ApproxcurveSizeMin=config["ApproxcurveSizeMin"].as<int>();
    ApproxcurveSizeMax=config["ApproxcurveSizeMax"].as<int>();
    InitHorizonLeftThreshold=config["InitHorizonLeftThreshold"].as<int>();
    InitHorizonRightThreshold=config["InitHorizonRightThreshold"].as<int>();
    CannyThreshold1=config["CannyThreshold1"].as<double>();
    CannyThreshold2=config["CannyThreshold2"].as<double>();
    CannyapertureSize=config["CannyapertureSize"].as<int>();
    PeaksIgnoreRadius=config["PeaksIgnoreRadius"].as<double>();

    YAML::Node object_points_yaml=config["object_points"];
    for(int i=0;i<8;i++){
        object_points.push_back(cv::Point3f(object_points_yaml[i][0].as<double>(),object_points_yaml[i][1].as<double>(),object_points_yaml[i][2].as<double>()));
    }

    YAML::Node redeem_front_points_yaml=config["redeem_front_points"];
    for(int i=0;i<4;i++){
        redeem_front_points.push_back(cv::Point3f(redeem_front_points_yaml[i][0].as<double>(),redeem_front_points_yaml[i][1].as<double>(),redeem_front_points_yaml[i][2].as<double>()));
    }

    camera_matrix=config["camera_matrix"].as<std::vector<double>>();
    if(camera_matrix.size()!=9){
        RCLCPP_ERROR(node_->get_logger(),"[loadConfig] camera_matrix size error!");
        return false;
    }
    camera_matrix_eigen<<camera_matrix[0],camera_matrix[1],camera_matrix[2],
        camera_matrix[3],camera_matrix[4],camera_matrix[5],
        camera_matrix[6],camera_matrix[7],camera_matrix[8];

    dist_coeffs=config["dist_coeffs"].as<std::vector<double>>();
    if(dist_coeffs.size()!=5){
        RCLCPP_ERROR(node_->get_logger(),"[loadConfig] dist_coeffs size error!");
        return false;
    }

    }
    catch(const YAML::Exception& e){
        RCLCPP_ERROR(node_->get_logger(),"[loadConfig] load config fail! %s",e.what());
        return false;
    }

    return true;
}

bool ArrowDetector::sortCorners(Counter& counter, std::vector<std::pair<cv::Point,cv::Point> > & end_points){

    const static std::vector<std::pair<int,int>> point_line_map={
        std::make_pair(0,2),
        std::make_pair(0,4),
        std::make_pair(1,3),
        std::make_pair(1,5),
        std::make_pair(2,3),
        std::make_pair(4,5)
    };

    if(counter.size()!=6){
        RCLCPP_ERROR(node_->get_logger(),"[sortCorners] counter size error!, get %ld",counter.size());
        return false;
    }

    // 最终答案
    Counter answer_counter(6);

    cv::Point2f center;
    float radius;

    cv::minEnclosingCircle(counter,center,radius);

    Counter2f triangle;

    cv::minEnclosingTriangle([&counter](){
        Counter2f ans;
        for(int i=0;i<6;i++){
            ans.push_back(counter[i]);
        }
        return ans;
    }(),triangle);

    // 将triangle和counter按照index进行匹配，由于最外侧点（也就是箭头尖尖上那个点是必然匹配成功的，其他侧边的不用管）
    // 前面是point_pairs的index是triangle的index，后面是counter的index
    std::vector<std::pair<int,double>> point_pairs(3,std::pair<int,double>(-1,1e9));
    // 在point_pairs中记录counter中已经被使用的点的index
    std::vector<bool> index_used(6,false);


    for(int i=0;i<3;i++){
        for(int e=0;e<6;e++){
            if(distance_points(triangle[i],counter[e])<point_pairs[i].second){
                point_pairs[i].second=distance_points(triangle[i],counter[e]);
                point_pairs[i].first=e;
            }
        }
        if(point_pairs[i].first==-1){
            RCLCPP_ERROR(node_->get_logger(),"[sortCorners] pair triangle and counter point fail!");
            return false;
        }
        index_used[point_pairs[i].first]=true;
    }

    // counter中最外侧点的index
    int top_point_index_counter=-1;
    // triangle中最外侧点的index
    int top_point_index_triangle=-1;
    // 记录两点中点到圆心的最小距离
    double min_center_dis=1e9;


    // 通过叉乘判断
    for(int i=0;i<3;i++){
        cv::Point2f center_point_another2=(triangle[(i+1)%3]+triangle[(i+2)%3])/2;

        if(distance_points(center,center_point_another2)<min_center_dis){
            min_center_dis=distance_points(center,center_point_another2);
            top_point_index_counter=point_pairs[i].first;
            top_point_index_triangle=i;
        }
    }

    {
        cv::Point2f main_vec=triangle[top_point_index_triangle]-center;
        cv::Point2f cross_vec=triangle[(top_point_index_triangle+1)%3]-center;
        answer_counter[0]=counter[top_point_index_counter];
        if(main_vec.cross(cross_vec)>=0){
            answer_counter[4]=counter[point_pairs[(top_point_index_triangle+1)%3].first];
            answer_counter[2]=counter[point_pairs[(top_point_index_triangle+2)%3].first];
        }
        else{
            answer_counter[2]=counter[point_pairs[(top_point_index_triangle+1)%3].first];
            answer_counter[4]=counter[point_pairs[(top_point_index_triangle+2)%3].first];
        }
    }

    // 在规定的角点顺序中的外侧三角形与内侧三角形的顶点的对应配对
    std::vector<std::pair<int,int>> index_pair={
        std::make_pair(0,1),
        std::make_pair(2,3),
        std::make_pair(4,5)
    };

    for(int i=0;i<3;i++){
        double min_distance=1e9;
        int min_index=-1;
        for(int e=0;e<6;e++){
            if(index_used[e]) continue;
            if(distance_points(answer_counter[index_pair[i].first],counter[e])<min_distance){
                min_distance=distance_points(answer_counter[index_pair[i].first],counter[e]);
                min_index=e;
            }
        }
        if(min_index==-1){
            RCLCPP_ERROR(node_->get_logger(),"[sortCorners] pair iner and outer counter point fail!");
            return false;
        }
        answer_counter[index_pair[i].second]=counter[min_index];
        index_used[min_index]=true;
    }

    counter=answer_counter;

    end_points.clear();
    for(int i=0;i<6;i++){
        end_points.push_back(std::make_pair(answer_counter[point_line_map[i].first],answer_counter[point_line_map[i].second]));
    }

    return true;

}

geometry_msgs::msg::TransformStamped ArrowDetector::reverseTransforme(
    const geometry_msgs::msg::TransformStamped& transform_A_to_child)
{
    // 1. 提取输入的旋转和平移
    tf2::Quaternion q_A_to_C;
    // tf2::fromMsg(transform_A_to_child.transform.rotation, q_A_to_C);
    q_A_to_C.setX(transform_A_to_child.transform.rotation.x);
    q_A_to_C.setY(transform_A_to_child.transform.rotation.y);
    q_A_to_C.setZ(transform_A_to_child.transform.rotation.z);
    q_A_to_C.setW(transform_A_to_child.transform.rotation.w);

    geometry_msgs::msg::Vector3 t_A_to_C = transform_A_to_child.transform.translation;

    // 2. 定义从 C 到 B 的旋转：绕 Z 轴逆时针 90 度 (-90度)
    // 可以使用 setRPY, setEulerZYX, 或者 setRotation
    // setRPY(roll, pitch, yaw) 这里的 yaw 是绕 Z 轴的旋转
    tf2::Quaternion q_C_to_B;
    q_C_to_B.setRPY(0, 0, -M_PI ); // 逆时针 90 度是 -90 度

    // 3. 计算从 A 到 B 的总旋转
    // q_A_to_B = q_A_to_C * q_C_to_B; (tf2 的乘法顺序)
    tf2::Quaternion q_A_to_B = q_A_to_C * q_C_to_B;

    // 4. 从 A 到 B 的平移与从 A 到 C 的平移相同
    geometry_msgs::msg::Vector3 t_A_to_B = t_A_to_C;

    // 5. 构造输出的 TransformStamped 消息
    geometry_msgs::msg::TransformStamped transform_A_to_B;

    transform_A_to_B.header.stamp = transform_A_to_child.header.stamp; // 使用相同的时间戳
    transform_A_to_B.header.frame_id = transform_A_to_child.header.frame_id; // A 坐标系
    transform_A_to_B.child_frame_id = transform_A_to_child.child_frame_id; // 给新的 B 坐标系命名

    transform_A_to_B.transform.translation = t_A_to_B;
    transform_A_to_B.transform.rotation.x = q_A_to_B.x();
    transform_A_to_B.transform.rotation.y = q_A_to_B.y();
    transform_A_to_B.transform.rotation.z = q_A_to_B.z();
    transform_A_to_B.transform.rotation.w = q_A_to_B.w();

    return transform_A_to_B;
}

ArrowDetector::~ArrowDetector(){}

}// Engineering_robot_Pnx