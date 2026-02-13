#ifndef GV_CAMERA_H
#define GV_CAMERA_H

#include <boost/filesystem.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>


#include <ctime>
#include <iostream>
#include <filesystem>
#include <vector>
#include <thread>
#include <Eigen/Dense>

#include "gv_d100/stereoDepth.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

enum GvStreamMode
{
    IR = 1,
    DEPTH = 2,
    COLOR = 3
};

class GvCamera : public rclcpp::Node {
public:
    GvCamera();

    ~GvCamera();

    void openCameraStream();

    void gvCameraInit();

    void depth2Pointcloud(const cv::Mat &depth, int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);

    sensor_msgs::msg::CameraInfo getCameraInfo(
        int idx, int streamMode);

private: 
    std::string mat_type2encoding(int mat_type);

private:
    typedef struct camera_topic {
        std::string pub_str_head_camera = "camera";
        std::string pub_str_depth = "/depth/image_raw";
        std::string pub_str_depth_align = "/depth/image_raw_d2c";
        std::string pub_str_depth_info = "/depth/camera_info";
        std::string pub_str_pointcloud = "/pointcloud";
        std::string pub_str_left = "/left/image_raw";
        std::string pub_str_left_camrea_info = "/left/camera_info";
        std::string pub_str_right = "/right/image_raw";
        std::string pub_str_right_camrea_info = "/right/camera_info";
        std::string pub_str_color = "/color/image_raw";
        std::string pub_str_color_info = "/color/camera_info";
    } CAMERA_TOPIC;
    CAMERA_TOPIC cameraTopic_;

    typedef struct dev_unit {
        image_transport::Publisher pub_depth;
        image_transport::Publisher pub_color;
        image_transport::Publisher pub_depth_d2c;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_left;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_right;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info;
    } DEV_UNIT;
    std::vector<DEV_UNIT> devs_;

private:
    std::mutex dataLock_;
    std::thread runThread_;

    // camera config
    int streamStatus_ = 5;
    bool publishPointCloud_ = false;
    bool alignD2c_ = false;
    float maxDistance_ = 4.0;
    int bandWidth_ = 3060;
    bool showImage_ = false;
    bool showDataSize_ = false;

    int devNum_;
    std::shared_ptr<gv::GvStereoDepth> gvD100_;
    std::vector<gv::GvStereoDepth::GV_CAMERA_PARAMS> gvParams_;

};
#endif
