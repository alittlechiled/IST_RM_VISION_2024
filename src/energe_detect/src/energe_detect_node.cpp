#include "energe_detect/energe_detect_node.hpp"

namespace EnergeDetect
{
    EnergeDetectNode::EnergeDetectNode(const rclcpp::NodeOptions &options):Node("EnergeDetecter", options)
    {
        Detecter = std::make_shared<EnergeDetecter>(EnergeDetecter(100,120,"red"));
        ThresholdGray = this->declare_parameter("ThresholdGray",125);
        ThresholdColor = this->declare_parameter("ThresholdColor",45);
        ThresholdKeyPoint = this->declare_parameter("ThresholdKeyPoint",110);
        this->InitDebug();
        ImgSub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
        this, "/image_raw", std::bind(&EnergeDetectNode::DetetctCallBack, this, std::placeholders::_1), 
        transport_, rmw_qos_profile_sensor_data));
        RCLCPP_INFO(this->get_logger(),"INIT SUCCESS!"); 
        
    }

    void EnergeDetectNode::DetetctCallBack(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        RCLCPP_INFO(this->get_logger(),"GET IMAGE!");

        ThresholdGray = get_parameter("ThresholdGray").as_int();
        ThresholdColor = get_parameter("ThresholdColor").as_int();
        ThresholdKeyPoint = get_parameter("ThresholdKeyPoint").as_int();
        
        Detecter->SetParams(ThresholdGray,ThresholdColor,ThresholdKeyPoint);
        RCLCPP_INFO(this->get_logger(),"UPDATE THE PARAM! ThresholdGray:%d,ThresholdColor:%d,ThresholdKeyPoint:%d",
                    ThresholdGray,ThresholdColor,ThresholdKeyPoint);

        auto img = cv_bridge::toCvShare(img_msg)->image;
        if(img.empty())
        {
            RCLCPP_INFO(this->get_logger(),"IMAGE IS EMPTY!");
            return;
        }
        else
        {
            this->Detecter->Detect(img);
            if(this->Debug)
            {
                BinaryImgPub.publish(cv_bridge::CvImage(img_msg->header, "mono8", this->Detecter->GetBinaryImg()).toImageMsg());
                ResualtImgPub.publish(cv_bridge::CvImage(img_msg->header, "mono8", this->Detecter->GetResultImg()).toImageMsg());
                DrawImgPub.publish(cv_bridge::CvImage(img_msg->header, "bgr8", this->Detecter->GetDrawImg()).toImageMsg());
                FinalImgPub.publish(cv_bridge::CvImage(img_msg->header, "bgr8", this->Detecter->GetFinalImg()).toImageMsg());
                BinaryColorImgPub.publish(cv_bridge::CvImage(img_msg->header, "mono8", this->Detecter->GetBinaryColorImg()).toImageMsg());
                KeyPointImgPub.publish(cv_bridge::CvImage(img_msg->header, "mono8", this->Detecter->GetKeyPointImg()).toImageMsg());
            }
        }
    }

    void EnergeDetectNode::InitDebug(void)
    {
        this->BinaryImgPub = image_transport::create_publisher(this,"/binary_image");
        this->ResualtImgPub = image_transport::create_publisher(this,"/resualt_image");
        this->BinaryColorImgPub = image_transport::create_publisher(this,"/binary_color_image");
        this->DrawImgPub = image_transport::create_publisher(this,"/draw_image");
        this->KeyPointImgPub = image_transport::create_publisher(this,"/key_point_image");
        this->FinalImgPub = image_transport::create_publisher(this,"/final_image");
    }

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(EnergeDetect::EnergeDetectNode)