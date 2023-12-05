#ifndef _ENERGE_DETECT_NODE_
#define _ENERGE_DETECT_NODE_

#include <rclcpp/parameter.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include "energe_detect/energe_detect.hpp"

namespace EnergeDetect
{
    class EnergeDetectNode:public rclcpp::Node
    {
        public:
            EnergeDetectNode(const rclcpp::NodeOptions &options);

        private:
            std::shared_ptr<EnergeDetecter> Detecter;
            std::shared_ptr<image_transport::Subscriber> ImgSub;
            
            image_transport::Publisher BinaryImgPub;
            image_transport::Publisher BinaryColorImgPub;
            image_transport::Publisher ResualtImgPub;
            image_transport::Publisher DrawImgPub;
            image_transport::Publisher KeyPointImgPub;
            image_transport::Publisher FinalImgPub;

            std::string transport_ = "raw";

            void InitDebug(void);
            void DetetctCallBack(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
            bool Debug = true;
            int ThresholdGray;
            int ThresholdColor;
            int ThresholdKeyPoint;

    };
}

#endif