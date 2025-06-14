#include "target_redeem_box/arrow_detector.hpp"
#include "target_redeem_box/utile.hpp"
#include <omp.h>


namespace Engineering_robot_Pnx{

ArrowDetector::ArrowDetector(const YAML::Node& config, 
        rclcpp::Node::SharedPtr node, 
        const std::string & name)
    :BaseDetector(config, node, name){
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
        }
    }

ArrowDetector::ArrowDetector(const YAML::Node& config, 
    const std::string & name):
    BaseDetector(config, name){
        bool ret = loadConfig();
        if(!ret){
            RCLCPP_ERROR(node_->get_logger(), " load config error");
        }
    }

void ArrowDetector::imagePreprocess(const cv::Mat & pre_image, cv::Mat & pos_image, cv::Mat & gray_image){
    // 以BGR通道为基准 储存每个通道的图像
    std::vector<cv::Mat> SplitImage;
    cv::split(pre_image,SplitImage);

    cv::Mat GreyImage(SplitImage[0].size(),SplitImage[0].type());

    cv::mixChannels(std::vector<cv::Mat>{SplitImage[0],SplitImage[2]},
        std::vector<cv::Mat>{GreyImage},
        std::vector<int>{
            0,0,
            1,0
    });

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
    std::mutex first_counters_mutex;

    #pragma omp parallel for
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
            std::lock_guard<std::mutex> lock(first_counters_mutex);
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
    std::mutex second_counters_mutex;

    #pragma omp parallel for
    for(int i=0;i<first_counters.size();i++){
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
            std::lock_guard<std::mutex> lock(second_counters_mutex);
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

    for(int i=0;i<second_counters_size.size();i++){
        if(second_counters_size[i]>max_pixel){
            max_pixel=second_counters_size[i];
            max_pixel_index=i;
        }
    }

    counter=second_counters[max_pixel_index];
    RCLCPP_INFO(node_->get_logger(), "[findCandidateContour] find %d contours success!",second_counters.size());

}

bool ArrowDetector::targetArrow(const cv::Mat & binary_image, const cv::Mat & gray_image, Counter2f& corners){
    Counter candidate_counter;
    if(!findCandidateContour(binary_image,gray_image,candidate_counter)){
        RCLCPP_INFO(node_->get_logger(), "[targetArrow] find candidate contours fail!");
        return false;
    }

    cv::drawContours(colored_image,Counters{candidate_counter},-1,cv::Scalar(432,32,255),2);

    // 直线拟合阶段



    // Counter2f subpix_counter;

    // for(int i=0;i<candidate_counter.size();i++){
    //     subpix_counter.push_back(cv::Point2f(candidate_counter[i].x,candidate_counter[i].y));
    // }

    // cv::TermCriteria cornerSubPix_criteria;
    // cv::cornerSubPix(gray_image,subpix_counter,cv::Size(5,5),cv::Size(-1,-1),cornerSubPix_criteria);

    // corners=subpix_counter;

    // RCLCPP_INFO(node_->get_logger(), "[targetArrow] find corners success!");

    return true;

}

bool ArrowDetector::getCounterCorners(const Counter& counter, 
    const cv::Mat& binary_image, 
    const cv::Mat& gray_image,
    Counter2f& corners){

    Counter approxcurve;
    std::vector<std::pair<cv::Point,cv::Point> > sorted_end_points;
    cv::approxPolyDP(counter,approxcurve,approxPolyDPEpsilon,1);

    // 对 approxcurve 进行排序，确定每个点的位置

    sortCorners(approxcurve, sorted_end_points);

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

    cv::minEnclosingCircle(approxcurve,center,radius);

    cv::circle(mask,cv::Point(center.x,center.y),radius,cv::Scalar(255),-1);

    cv::copyTo(binary_image, masked_image, mask);

    }

    cv::Canny(masked_image, canny_image, CannyThreshold1, CannyThreshold2, CannyapertureSize);

    Counters point_sets;
    std::vector<std::pair<cv::Point,cv::Point> > end_points;
    find_polygon_counter_points_sets(canny_image, approxcurve, PeaksIgnoreRadius, point_sets, end_points);

    if(point_sets.size()!=6||end_points.size()!=6){
        RCLCPP_ERROR(node_->get_logger(),"[getCounterCorners] size of point_sets or end_points != 6");
        return false;
    }

    // cv::fitLine(LinesPoints[i],line,cv::DIST_L2,0,0.01,0.01);

    // 符合sorted_end_points顺序的直线
    std::vector<cv::Vec4d> sorted_fitted_lines(6);

    // 拟合的直线
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

    

}

bool ArrowDetector::detect(InputData input_data, DetectorOutput& output_data){

    output_data.result_image_=std::make_shared<cv::Mat>(input_data.image.clone());

    colored_image = *output_data.result_image_;

    cv::Mat binary_image;
    cv::Mat gray_image;

    imagePreprocess(colored_image,binary_image,gray_image);

    Counter2f corners;

    if(!targetArrow(binary_image,gray_image,corners)){
        RCLCPP_INFO(node_->get_logger(), "[detect] find corners fail!");
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;

    bool pnp_solver_success=pnp_solver(corners,
        object_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        0,
        cv::SOLVEPNP_IPPE);
    
    if(!pnp_solver_success){
        RCLCPP_INFO(node_->get_logger(), "[detect] solve pnp fail!");
        return false;
    }

    // 画出结果

    draw_pnp_result(colored_image,rvec,tvec,camera_matrix,object_points,cv::Scalar(32,43,132),2,cv::Point(0,0),true);

    output_data.rvec=rvec;
    output_data.tvec=tvec;
    return true;
}

}// Engineering_robot_Pnx