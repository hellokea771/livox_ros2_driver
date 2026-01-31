#include <memory>
#include <mutex>
#include <cmath>
#include <vector>
#include <iomanip>

// ROS 2 核心头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 消息转换与时间同步
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// PCL 与 OpenCV
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
// #include <omp.h> // 暂时注释掉 OpenMP，先跑通逻辑

class LidarCameraFusionNode : public rclcpp::Node {
public:
    LidarCameraFusionNode() : Node("lidar_camera_fusion_node") {
        
        this->declare_parameter<std::string>("img_topic", "/image_raw");
        this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        
        std::string img_topic_name = this->get_parameter("img_topic").as_string();
        std::string lidar_topic_name = this->get_parameter("lidar_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "订阅图像话题: %s", img_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅雷达话题: %s", lidar_topic_name.c_str());

        // ==================== [参数初始化：长焦模式] ====================
        
        // 1. 内参 (Intrinsics): 使用原始长焦标定值
        fx = 13800.0; 
        fy = 13800.0; 
        cx = 743.785;  
        cy = 566.278; 
        
        // 2. 畸变系数 (Distortion): 强制清零
        // 标定值 -72 在此处不可用，长焦镜头本身畸变很小，为了能投影成功，我们忽略畸变。
        k1 = 0.0; k2 = 0.0; p1 = 0.0; p2 = 0.0; 

        // 3. 外参 Rcl (Lidar -> Camera): 直接使用标定值
        // 矩阵分析结果确认：这就是 Lidar 到 Camera 的变换，不要转置。
        r[0] =  0.017659; r[1] = -0.999684; r[2] =  0.017928;
        r[3] = -0.029468; r[4] = -0.018444; r[5] = -0.999396;
        r[6] =  0.999410; r[7] =  0.017120; r[8] = -0.029784;

        // 4. 外参 Pcl (平移): 直接使用标定值
        // 修改这里的数值进行微调
        t[0] = -0.275539;  // +右, -左
        t[1] =  0.686930;  // +下, -上
        t[2] = -0.230909;  // +前, -后 (一般不用动)

        // 过滤阈值
        min_dist = 0.5; max_dist = 60.0;
        // ==============================================================

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

        image_sub_.subscribe(this, img_topic_name, qos);
        cloud_sub_.subscribe(this, lidar_topic_name, qos);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), image_sub_, cloud_sub_);
        
        sync_->registerCallback(
            std::bind(&LidarCameraFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        colored_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/colored_points", 10);
    }


private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;

    double fx, fy, cx, cy, k1, k2, p1, p2;
    double r[9], t[3];
    double min_dist, max_dist;

    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
        
        // ==================== [时间戳差异检查] ====================
        auto img_stamp = rclcpp::Time(img_msg->header.stamp);
        auto cloud_stamp = rclcpp::Time(cloud_msg->header.stamp);
        auto current_time = this->get_clock()->now();
        
        double time_diff_ms = std::abs(img_stamp.seconds() - cloud_stamp.seconds()) * 1000.0;
        double img_delay_ms = (current_time.seconds() - img_stamp.seconds()) * 1000.0;
        double cloud_delay_ms = (current_time.seconds() - cloud_stamp.seconds()) * 1000.0;
        
        // 每次都输出时间戳差异信息
        RCLCPP_INFO(this->get_logger(), 
            "=== 时间戳检查 ===\n"
            "相机时间戳: %u.%09u\n"
            "雷达时间戳: %u.%09u\n"
            "时间戳差异: %.3f ms\n"
            "相机延迟: %.3f ms\n"
            "雷达延迟: %.3f ms",
            img_msg->header.stamp.sec, img_msg->header.stamp.nanosec,
            cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec,
            time_diff_ms, img_delay_ms, cloud_delay_ms);
        
        // 时间戳差异过大警告
        if (time_diff_ms > 100.0) {  // 超过100ms
            RCLCPP_WARN(this->get_logger(), "警告: 时间戳差异过大 (%.3f ms)，可能影响融合质量!", time_diff_ms);
        } else if (time_diff_ms > 50.0) {  // 超过50ms
            RCLCPP_WARN(this->get_logger(), "注意: 时间戳差异较大 (%.3f ms)", time_diff_ms);
        } else {
            RCLCPP_INFO(this->get_logger(), "时间同步良好 (差异: %.3f ms)", time_diff_ms);
        }
        // ===================================================
        
        // --- 手动包装 Image ---
        cv::Mat current_img_raw;
        int type;
        // 强制检查编码
        if (img_msg->encoding == "bgr8") type = CV_8UC3;
        else if (img_msg->encoding == "rgb8") type = CV_8UC3;
        else if (img_msg->encoding == "mono8") type = CV_8UC1;
        else {
            return; // 暂不支持其他格式
        }

        cv::Mat wrapped_img(img_msg->height, img_msg->width, type, 
                            const_cast<unsigned char*>(img_msg->data.data()), img_msg->step);

        if (wrapped_img.empty()) return;

        // 克隆一份数据，避免多线程访问冲突，且如果原图是 RGB，这里默认当做 BGR 处理（可能会颜色反转，但不会崩）
        cv::Mat current_img = wrapped_img.clone(); 

        // 2. 点云转换
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *raw_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // 这里只是预留空间
        colored_cloud->points.reserve(raw_cloud->size());

        int w = current_img.cols;
        int h = current_img.rows;
        
        // 修正 2：改回基于索引的循环，这样就有变量 i 了
        for (size_t i = 0; i < raw_cloud->size(); ++i) {
            const auto& pt = raw_cloud->points[i];
            
            pcl::PointXYZRGB pt_rgb;
            pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
            
            // 默认颜色
            pt_rgb.r = 100; pt_rgb.g = 100; pt_rgb.b = 100;

            if (!std::isfinite(pt.x) || pt.x < min_dist || pt.x > max_dist) {
                colored_cloud->push_back(pt_rgb);
                continue;
            }

            // 变换
            double xc = r[0]*pt.x + r[1]*pt.y + r[2]*pt.z + t[0];
            double yc = r[3]*pt.x + r[4]*pt.y + r[5]*pt.z + t[1];
            double zc = r[6]*pt.x + r[7]*pt.y + r[8]*pt.z + t[2];

            if (zc <= 0.1) {
                colored_cloud->push_back(pt_rgb);
                continue;
            }

            // 投影
            double inv_z = 1.0 / zc;
            double xn = xc * inv_z;
            double yn = yc * inv_z;
            double r2 = xn*xn + yn*yn;
            double r4 = r2*r2;
            double rad = 1.0 + k1*r2 + k2*r4;
            double x_d = xn * rad + 2*p1*xn*yn + p2*(r2 + 2*xn*xn);
            double y_d = yn * rad + p1*(r2 + 2*yn*yn) + 2*p2*xn*yn;

            int u = static_cast<int>(fx * x_d + cx);
            int v = static_cast<int>(fy * y_d + cy);

            // 调试打印：现在变量 i 存在了，不会报错


            if (u >= 0 && u < w && v >= 0 && v < h) {
                if (type == CV_8UC3) {
                    const cv::Vec3b& pixel = current_img.at<cv::Vec3b>(v, u);
                    if (img_msg->encoding == "rgb8") {
                         pt_rgb.r = pixel[0]; pt_rgb.g = pixel[1]; pt_rgb.b = pixel[2];
                    } else { // bgr8
                         pt_rgb.r = pixel[2]; pt_rgb.g = pixel[1]; pt_rgb.b = pixel[0];
                    }
                } else {
                    // 灰度图
                    uint8_t val = current_img.at<uint8_t>(v, u);
                    pt_rgb.r = val; pt_rgb.g = val; pt_rgb.b = val;
                }
            }
            colored_cloud->push_back(pt_rgb);
        }

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*colored_cloud, out_msg);
        out_msg.header = cloud_msg->header;
        colored_cloud_pub_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "融合完成，输出 %zu 个着色点", colored_cloud->size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarCameraFusionNode>());
    rclcpp::shutdown();
    return 0;
}