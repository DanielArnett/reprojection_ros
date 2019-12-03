#ifndef SIMPLE_RENDERER_NODECORE_H
#define SIMPLE_RENDERER_NODECORE_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "simple_renderer.h"
#include <dynamic_reconfigure/server.h>
#include <reprojection/reprojectionConfig.h>

namespace opengl_ros {

class SimpleRendererNode
{
    //Handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //Publishers and Subscribers
    image_transport::Publisher imagePublisher_;
    image_transport::Subscriber imageSubscriber_;
    
    //Other members
    std::unique_ptr<cgs::SimpleRenderer> renderer_;
    cv::Mat output_;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void reconfigure_callback(reprojection::reprojectionConfig &config, uint32_t level);
    float correction1, correction2, correction3, correction4;
    float croppedWidth, croppedHeight, xCenter, yCenter;
    float pitch, roll, yaw, x, y, z, fovIn, fovOut, blendFront, blendBack;
    int inputProjection, outputProjection, gridLines;


public:
    SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
