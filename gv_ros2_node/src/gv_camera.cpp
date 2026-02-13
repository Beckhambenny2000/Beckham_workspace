#include "gv_camera.h"
#include <opencv2/opencv.hpp>
using namespace gv;

GvCamera::GvCamera() : rclcpp::Node("gv_camera") {   
    
    RCLCPP_INFO_STREAM(this->get_logger(), "********************************************************************");
    RCLCPP_INFO_STREAM(this->get_logger(), "************************ SHOW CONFIG PARAMS ************************");
    RCLCPP_INFO_STREAM(this->get_logger(), "********************************************************************");
    auto yamlPath = this->declare_parameter<std::string>("config_path", "") + "/camera_config.yaml";
    try {
        RCLCPP_INFO_STREAM(this->get_logger(), "load  GV camera  configuration file: " << yamlPath);
        auto config = YAML::LoadFile(yamlPath);
        if (config["stream_status"].IsDefined())
            streamStatus_ = config["stream_status"].as<int>();
        if (config["publish_pointcloud"].IsDefined())
           publishPointCloud_ = config["publish_pointcloud"].as<bool>();
        if (config["align_depth_to_color"].IsDefined())
           alignD2c_ = config["align_depth_to_color"].as<bool>();
        if (config["band_width"].IsDefined())
            bandWidth_ = config["band_width"].as<int>();
        if (config["max_distance"].IsDefined())
           maxDistance_ = config["max_distance"].as<float>();
        if (config["show_image"].IsDefined())
           showImage_ = config["show_image"].as<bool>();
        if (config["show_data_size"].IsDefined())
           showDataSize_ = config["show_data_size"].as<bool>();
        
        RCLCPP_INFO_STREAM(this->get_logger(), " stream status : " << streamStatus_);
        RCLCPP_INFO_STREAM(this->get_logger(), " publish pointcloud : " << publishPointCloud_);
        RCLCPP_INFO_STREAM(this->get_logger(), " align depth to color : " << alignD2c_);
        RCLCPP_INFO_STREAM(this->get_logger(), " band width  : " << bandWidth_);
        RCLCPP_INFO_STREAM(this->get_logger(), " max distance : " << maxDistance_);
        RCLCPP_INFO_STREAM(this->get_logger(), " show image : " << showImage_);
        RCLCPP_INFO_STREAM(this->get_logger(), " show data size : " << showDataSize_);

    } catch (std::exception) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot find topic info  configuration file: " << yamlPath << ". Use default param");
        exit(-1);
    }
    //device init
    RCLCPP_INFO_STREAM(this->get_logger(), "******* START GV CAMERA INIT ********");
    gvCameraInit();
    RCLCPP_INFO_STREAM(this->get_logger(), "******* FINISH GV CAMERA INIT ********");

    // ex;
    RCLCPP_INFO_STREAM(this->get_logger(), "dev num : " << devNum_);
    if (devNum_ > 0) {
        devs_.resize(devNum_);
        for(int i = 0; i < static_cast<int>(devNum_); i++) {
            int n = i;
            std::string pub_str_camera_local = cameraTopic_.pub_str_head_camera + std::to_string(n);

           
            std::string pub_str_head_left_local = pub_str_camera_local + cameraTopic_.pub_str_left;
            std::string pub_str_head_right_local = pub_str_camera_local + cameraTopic_.pub_str_right;
            std::string pub_str_head_left_camera_info_local = pub_str_camera_local + cameraTopic_.pub_str_left_camrea_info;          
            std::string pub_str_head_right_camera_info_local = pub_str_camera_local + cameraTopic_.pub_str_right_camrea_info;
            devs_[n].pub_ir_left = this->create_publisher<sensor_msgs::msg::Image>(pub_str_head_left_local.c_str(), rclcpp::SensorDataQoS());
            devs_[n].pub_ir_right = this->create_publisher<sensor_msgs::msg::Image>(pub_str_head_right_local.c_str(), rclcpp::SensorDataQoS());
              
            devs_[n].pub_left_camera_info = this->create_publisher<sensor_msgs::msg::CameraInfo>(pub_str_head_left_camera_info_local.c_str(), rclcpp::SensorDataQoS());
            devs_[n].pub_right_camera_info = this->create_publisher<sensor_msgs::msg::CameraInfo>(pub_str_head_right_camera_info_local.c_str(), rclcpp::SensorDataQoS());
            
            std::string pub_str_head_depth_local = pub_str_camera_local + cameraTopic_.pub_str_depth;
            std::string pub_str_head_depth_align_local = pub_str_camera_local + cameraTopic_.pub_str_depth_align;
            std::string pub_str_head_depth_info_local = pub_str_camera_local + cameraTopic_.pub_str_depth_info;
            devs_[n].pub_depth = image_transport::create_publisher(this, pub_str_head_depth_local.c_str());
            devs_[n].pub_depth_d2c = image_transport::create_publisher(this, pub_str_head_depth_align_local.c_str());
            devs_[n].pub_depth_info = this->create_publisher<sensor_msgs::msg::CameraInfo>(pub_str_head_depth_info_local.c_str(), rclcpp::SensorDataQoS());
            if(publishPointCloud_)
            {
                std::string pub_str_head_pointcloud_local = pub_str_camera_local + cameraTopic_.pub_str_pointcloud;
                devs_[n].pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_str_head_pointcloud_local.c_str(), 1);
                //devs_[n].pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_str_head_pointcloud_local.c_str(), rclcpp::SensorDataQoS());
            }
        
            std::string pub_str_head_color_local = pub_str_camera_local + cameraTopic_.pub_str_color;
            std::string pub_str_head_color_info_local = pub_str_camera_local + cameraTopic_.pub_str_color_info;
            devs_[n].pub_color = image_transport::create_publisher(this, pub_str_head_color_local.c_str());
            devs_[n].pub_color_info = this->create_publisher<sensor_msgs::msg::CameraInfo>(pub_str_head_color_info_local.c_str(), rclcpp::SensorDataQoS());
            
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "device is not set !");
        exit(-1);
    }    

    runThread_ = std::thread(&GvCamera::openCameraStream, this);
}

GvCamera::~GvCamera() {
    RCLCPP_INFO_STREAM(this->get_logger(), "~GvCamera");
    runThread_.join();
    //quit();
}

void GvCamera::openCameraStream()
{
    while(1)
    {
            for(int idx = 0; idx < devNum_; idx++)
            {
                //std::vector<unsigned char> alignDepthData = gvD100_->getAlignedDepthData(idx);
                auto curStamp = rclcpp::Clock().now();
                if(streamStatus_ == 1)
                {
                    cv::Mat irImage = cv::Mat::zeros(400 * 2, 640, CV_8UC1);
                    std::vector<unsigned char>  irData = gvD100_->getIrStreamData(idx);
                    
                    irImage = cv::Mat(400 * 2, 640,CV_8UC1, &irData[0]); 
                    cv::Mat left_ir, right_ir;
                    //ir format gray8 640 * 400 * (2)
                    cv::Rect rect_left(0, 0, 640, 400);
                    cv::Rect rect_right(0, 400, 640, 400);

                    irImage(rect_left).copyTo(left_ir);
                    irImage(rect_right).copyTo(right_ir);
                    if(showDataSize_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "irData size : " << irData.size());

                    if(showImage_)
                    {
                        cv::imshow("irImage_" + std::to_string(idx) , irImage);
                        cv::waitKey(1);
                    }

                    sensor_msgs::msg::Image::UniquePtr msg_ir_left(new sensor_msgs::msg::Image());
                    sensor_msgs::msg::Image::UniquePtr msg_ir_right(new sensor_msgs::msg::Image());
                    
                    msg_ir_left->header.stamp = curStamp;
                    msg_ir_right->header.stamp = curStamp;
                
                    if((!left_ir.empty()) && (!right_ir.empty()))
                    {
                        sensor_msgs::msg::CameraInfo irCamInfo;
                        {
                            std::unique_lock<std::mutex> camDataLock(dataLock_, std::defer_lock);
                            irCamInfo = getCameraInfo(idx, GvStreamMode::IR);
                        }
                        irCamInfo.header.stamp = curStamp;
                        irCamInfo.header.frame_id = "camera_link";
                        if(devs_[idx].pub_left_camera_info->get_subscription_count() > 0)
                            devs_[idx].pub_left_camera_info->publish(irCamInfo);

                        if(devs_[idx].pub_right_camera_info->get_subscription_count() > 0)
                            devs_[idx].pub_right_camera_info->publish(irCamInfo);


                        msg_ir_left->header.stamp = curStamp;
                        msg_ir_left->header.frame_id = "camera_link";
                        msg_ir_left->height = left_ir.rows;
                        msg_ir_left->width = left_ir.cols;
                        msg_ir_left->encoding = mat_type2encoding(left_ir.type());
                        msg_ir_left->is_bigendian = false;
                        msg_ir_left->step = static_cast<sensor_msgs::msg::Image::_step_type>(left_ir.step);
                        msg_ir_left->data.assign(left_ir.datastart, left_ir.dataend);
                        
                        if(devs_[idx].pub_ir_left->get_subscription_count()>0)
                            devs_[idx].pub_ir_left->publish(std::move(msg_ir_left));

                        msg_ir_right->header.stamp = curStamp;
                        msg_ir_right->header.frame_id = "camera_link";
                        msg_ir_right->height = right_ir.rows;
                        msg_ir_right->width = right_ir.cols;
                        msg_ir_right->encoding = mat_type2encoding(right_ir.type());
                        msg_ir_right->is_bigendian = false;
                        msg_ir_right->step = static_cast<sensor_msgs::msg::Image::_step_type>(right_ir.step);
                        msg_ir_right->data.assign(right_ir.datastart, right_ir.dataend);
                        
                        if(devs_[idx].pub_ir_right->get_subscription_count()>0)
                            devs_[idx].pub_ir_right->publish(std::move(msg_ir_right)); 

                        
                    }

                }
                if(streamStatus_ == 3)
                { 
                    std::vector<unsigned char> colorData = gvD100_->getColorStreamData
                    (idx);
                    if(showDataSize_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "color data size : " << colorData.size());
                    if(colorData.size() > 0)
                    {
                        cv::Mat colorImage = cv::imdecode(cv::Mat(colorData), cv::IMREAD_COLOR);
                        if(showDataSize_)
                            RCLCPP_INFO_STREAM(this->get_logger(), "color image test size : " << colorImage.size());
                        if(showImage_)
                        {
                            if(!colorImage.empty())
                                cv::imshow("color_image" + std::to_string(idx), colorImage);
                            cv::waitKey(1);
                        }
                    }
                    
                }
                else if(streamStatus_ == 5)
                {
                    cv::Mat depthImage = cv::Mat::zeros(400, 640, CV_16UC1); 
                    std::vector<unsigned char> depthData = gvD100_->getDepthStreamData(idx);
                    if(showDataSize_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "depth data size : " << depthData.size());

                    if(depthData.size() > 0)
                    {
                        depthImage = cv::Mat(400, 640,CV_16UC1, &depthData[0]); 
                        if(showImage_)
                        {
                            cv::Mat depthColor, depth8U;
                            depthImage.convertTo(depth8U, CV_8U, 255. / 32 / 32);
                            double min, max;
                            cv::minMaxIdx(depth8U, &min, &max);
                            cv::Mat adjMap;
                            cv::convertScaleAbs(depth8U, adjMap, 255 / max);
                            cv::applyColorMap(adjMap, depthColor, cv::COLORMAP_JET);
                            cv::imshow("depthImage_" + std::to_string(idx), depthColor);
                            cv::waitKey(1);
                        }

                        sensor_msgs::msg::CameraInfo depthCamInfo;
                        {
                            std::unique_lock<std::mutex> camDataLock(dataLock_, std::defer_lock);
                            depthCamInfo = getCameraInfo(idx, GvStreamMode::DEPTH);
                        }
                        depthCamInfo.header.stamp = curStamp;
                        depthCamInfo.header.frame_id = "camera_link";              
                        if(devs_[idx].pub_depth_info->get_subscription_count() > 0)
                            devs_[idx].pub_depth_info->publish(depthCamInfo);
                       
                        sensor_msgs::msg::Image::Ptr depth_ros = 
                            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depthImage).toImageMsg();
                        
                        depth_ros->header.stamp = curStamp;
                        depth_ros->header.frame_id = "camera_link";

                        if (devs_[idx].pub_depth.getNumSubscribers() > 0) {
                            devs_[idx].pub_depth.publish(depth_ros);
                        }

                        if(publishPointCloud_)
                        {
                            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
                            depth2Pointcloud(depthImage, idx, pointcloud);
                            
                            sensor_msgs::msg::PointCloud2 pointCloudMsg;
                            pcl::toROSMsg(*pointcloud, pointCloudMsg);
                            pointCloudMsg.header.frame_id = "camera_link";
                            pointCloudMsg.header.stamp = curStamp;
                            if (devs_[idx].pub_pointcloud->get_subscription_count() > 0)
                                devs_[idx].pub_pointcloud->publish(pointCloudMsg);
                            
                            if(showDataSize_)
                                RCLCPP_INFO_STREAM(this->get_logger(), "point cloud data size : " << pointcloud->points.size());

                            if(showImage_)
                            {
                                if(pointcloud->points.size() > 0)
                                {   
                                    pcl::PCDWriter writerPcd;
                                    writerPcd.writeBinaryCompressed("pointcloud.pcd", *pointcloud);
                                }
                            }
                        }

                        
                    }
                    std::vector<unsigned char> colorData = gvD100_->getColorStreamData
                    (idx);
                    if(showDataSize_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "color data size : " << colorData.size());

                    cv::Mat colorImageVga = cv::Mat::zeros(400, 640, CV_8UC3);
                    if(colorData.size() > 0)
                    {
                        colorImageVga = cv::imdecode(cv::Mat(colorData), cv::IMREAD_COLOR);
                        if(!colorImageVga.empty())
                        {
                            sensor_msgs::msg::CameraInfo colorCamInfo;
                            {
                                std::unique_lock<std::mutex> camDataLock(dataLock_, std::defer_lock);
                                colorCamInfo = getCameraInfo(idx, GvStreamMode::COLOR);
                            }

                            colorCamInfo.header.stamp = curStamp;
                            colorCamInfo.header.frame_id = "camera_link";              
                            if(devs_[idx].pub_color_info->get_subscription_count() > 0)
                                devs_[idx].pub_color_info->publish(colorCamInfo);
                            if(showImage_)
                            {
                                cv::imshow("color_vga" + std::to_string(idx), colorImageVga);
                                cv::waitKey(1);
                            }
                            // sensor_msgs::msg::Image::UniquePtr msg_color(new sensor_msgs::msg::Image());
                            // msg_color->header.frame_id = "camera_link";
                            // msg_color->header.stamp = curStamp;
                            // msg_color->height = colorImageVga.rows;
                            // msg_color->width = colorImageVga.cols;
                            // msg_color->encoding = mat_type2encoding(colorImageVga.type());
                            // msg_color->is_bigendian = false;
                            // msg_color->step = static_cast<sensor_msgs::msg::Image::_step_type>(colorImageVga.step);
                            // msg_color->data.assign(colorImageVga.datastart, colorImageVga.dataend);

                            sensor_msgs::msg::Image::Ptr color_ros = 
                            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC3, colorImageVga).toImageMsg();
                                        
                            if(devs_[idx].pub_color.getNumSubscribers() > 0)
                                devs_[idx].pub_color.publish(color_ros); // Publish.
                        }
                       

                    }
                    if(alignD2c_)
                    {
                        std::vector<double> alignDepthData;
                        std::vector<unsigned char> alignColorData;
                        gvD100_->getAlignedDepthData(idx, alignDepthData, alignColorData);
                        if((!alignDepthData.empty()) && (!alignColorData.empty()))
                        {
                            cv::Mat alignDepthImage = cv::Mat(400, 640,CV_64FC1, &alignDepthData[0]); 
                            cv::Mat alignColorImage = cv::imdecode(cv::Mat(alignColorData), cv::IMREAD_COLOR);
                            if(showImage_)
                            {
                                cv::imshow("align_depth_" + std::to_string(idx), alignDepthImage);
                                cv::imshow("align_color_" + std::to_string(idx), alignColorImage);
                                cv::waitKey(1);
                            }
                            sensor_msgs::msg::Image::Ptr depth_align_ros = 
                            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_64FC1, alignDepthImage).toImageMsg();
                        
                            depth_align_ros->header.stamp = curStamp;
                            depth_align_ros->header.frame_id = "camera_link";

                            if (devs_[idx].pub_depth_d2c.getNumSubscribers() > 0) {
                                devs_[idx].pub_depth_d2c.publish(depth_align_ros);
                            }
                        }
                    }
                } 
            //}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

sensor_msgs::msg::CameraInfo GvCamera::getCameraInfo(int idx, int streamMode) 
{
    sensor_msgs::msg::CameraInfo cam;
    cam.height = 400;
    cam.width = 640;
    cam.distortion_model = "plumb_bob";
    if((streamMode == GvStreamMode::IR) || (streamMode == GvStreamMode::DEPTH))
    {
        //(new sensor_msgs::msg::CameraInfo());
        std::vector<double> camD{0., 0., 0., 0., 0,};

        std::array<double, 9> camK = {
                gvParams_[idx].fx,  0,  gvParams_[idx].cx,
                0, gvParams_[idx].fy, gvParams_[idx].cy,
                0, 0, 1};

        std::array<double, 12> camP = {
            gvParams_[idx].fx,  0.,  gvParams_[idx].cx, 0.,
            0., gvParams_[idx].fy,  gvParams_[idx].cy, 0.,
            0., 0.,  1., 0.};

        std::array<double, 9> camR = {
            1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.};

        cam.d = camD;
        cam.k = camK;
        cam.p = camP;
        cam.r = camR;
    }
    else if(streamMode == GvStreamMode::COLOR)
    {
        std::vector<double> camD{
            gvParams_[idx].rightRGBDistCoeffs[0],
            gvParams_[idx].rightRGBDistCoeffs[1],
            gvParams_[idx].rightRGBDistCoeffs[2],
            gvParams_[idx].rightRGBDistCoeffs[3],
            gvParams_[idx].rightRGBDistCoeffs[4]};

        std::array<double, 9> camK = {
            gvParams_[idx].rightRGBCameraMatrix[0],  0,  gvParams_[idx].rightRGBCameraMatrix[2],
            0, gvParams_[idx].rightRGBCameraMatrix[4], gvParams_[idx].rightRGBCameraMatrix[5],
            0, 0, 1};

        std::array<double, 12> camP = {
            gvParams_[idx].rightRGBCameraMatrix[0],  0,  gvParams_[idx].rightRGBCameraMatrix[2], 0.,
            0, gvParams_[idx].rightRGBCameraMatrix[4], gvParams_[idx].rightRGBCameraMatrix[5], 0.,
            0., 0.,  1., 0.};

        std::array<double, 9> camR = {
            1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.};
       
        cam.d = camD;
        cam.k = camK;
        cam.p = camP;
        cam.r = camR;
    }

   
    return cam;
}


std::string GvCamera::mat_type2encoding(int mat_type) {
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

void GvCamera::gvCameraInit()
{
    gvD100_ = std::make_shared<gv::GvStereoDepth>();      
    if(gvD100_->initDeivce())
    {
        devNum_ = gvD100_->getDeviceNum();
        RCLCPP_INFO_STREAM(this->get_logger(), "camera number : " << devNum_);
        if(devNum_ > 0)
        {
            for(int idx = 0; idx < devNum_; idx++)
            {
                if(gvD100_->enableCamera(idx, true, bandWidth_, 3, 0))
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "stereo depth device init success !");
                    gvD100_->setGvDeviceStreamStatus(idx, streamStatus_);
                }
            }
        }

        for(int idx = 0; idx < devNum_; idx++)
        {
            gvParams_ =  gvD100_->getGvCameraParams();
            RCLCPP_INFO_STREAM(this->get_logger(), " *** get stereo depth params *** ");
            RCLCPP_INFO_STREAM(this->get_logger(), "fx : " << gvParams_[idx].fx);
            RCLCPP_INFO_STREAM(this->get_logger(), "fy : " << gvParams_[idx].fy);
            RCLCPP_INFO_STREAM(this->get_logger(), "cx : " << gvParams_[idx].cx);
            RCLCPP_INFO_STREAM(this->get_logger(), "cy : " << gvParams_[idx].cy);
            RCLCPP_INFO_STREAM(this->get_logger(), "baseline : " << gvParams_[idx].baseline);
        
            RCLCPP_INFO_STREAM(this->get_logger(), " *** get vga color camera matrix *** ");
            for(int n = 0; n < 9; n++)
                RCLCPP_INFO_STREAM(this->get_logger(), gvParams_[idx].rightRGBCameraMatrix[n] << " ");
            RCLCPP_INFO_STREAM(this->get_logger(),  " *** get vga color distcoeffs *** ");
            for(int n = 0; n < 5; n++)
                RCLCPP_INFO_STREAM(this->get_logger(), gvParams_[idx].rightRGBDistCoeffs[n] << " ");
        }
    }   
}

void GvCamera::depth2Pointcloud(const cv::Mat &depthImage, int idx,  pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
    cv::Mat image = depthImage.clone();
    double fx =  gvParams_[idx].fx;
    double fy =  gvParams_[idx].fy;
    double cx =  gvParams_[idx].cx;
    double cy =  gvParams_[idx].cy;
    if(alignD2c_)
    {
        fx =  gvParams_[idx].rightRGBCameraMatrix[0];
        fy =  gvParams_[idx].rightRGBCameraMatrix[4];
        cx =  gvParams_[idx].rightRGBCameraMatrix[2];
        cy =  gvParams_[idx].rightRGBCameraMatrix[5];
    }
    //RCLCPP_INFO_STREAM(this->get_logger(), "camera info : " << fx << " " << fy << " " << cx << " " << cy);
    float scale = 1000.;
    if(alignD2c_)
        scale = 1;
    int height = image.rows;
    int width = image.cols;
    for (int r = 0; r < height; r++)
    {
        ushort *depthPtr = image.ptr<ushort>(r);
        // cv::Vec3f *depthPtr = depth32f.ptr<cv::Vec3f>(r);
        for (int c = 0; c < width; c++)
        {
            float depth = depthPtr[c] / scale;
            if (depth > maxDistance_)
                continue;
            pcl::PointXYZ point;
            point.x = (c - cx) / fx * depth;
            point.y = (r - cy) / fy * depth;
            point.z = depth;
            pointcloud->points.emplace_back(point);
        }
    }
    pointcloud->width = pointcloud->points.size();
    pointcloud->height = 1;
    pointcloud->is_dense = true;
}
