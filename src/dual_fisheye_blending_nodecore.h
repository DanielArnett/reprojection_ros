#ifndef SIMPLE_RENDERER_NODECORE_H
#define SIMPLE_RENDERER_NODECORE_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "simple_renderer.h"
#include <dynamic_reconfigure/server.h>
#include <reprojection/dual_fisheye_blendingConfig.h>

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
    std::unique_ptr<cgs::SimpleRenderer> left_renderer_;
    std::unique_ptr<cgs::SimpleRenderer> right_renderer_;
    std::unique_ptr<cgs::SimpleRenderer> right_rotated_renderer_;
    std::unique_ptr<cgs::SimpleRenderer> blending_renderer_;
    cv::Mat left_output_;
    cv::Mat right_output_;
    cv::Mat blended_output_;

    int width_, height_;
    std::string vertexShader_, reprojectionFragmentShader_, blendingFragmentShader_;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void reconfigure_callback(reprojection::dual_fisheye_blendingConfig &config, uint32_t level);

    struct re_config {
        float correction1, correction2, correction3, correction4;
        float croppedWidth, croppedHeight, xCenter, yCenter;
        float pitch, roll, yaw, x, y, z, fovIn, fovOut, blendFront, blendBack;
        int inputProjection, outputProjection, gridLines, blendImages, linearBlend;
    } left_recon_, right_recon_, rotated_recon_, blended_recon_;
    void set_reprojection_params(reprojection::dual_fisheye_blendingConfig &config, re_config recon, const std::unique_ptr<cgs::SimpleRenderer>& renderer);
    void setUniforms(std::unique_ptr<cgs::SimpleRenderer>& renderer, re_config& config);

public:
    SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void run();
};

} //opengl_ros

#endif
