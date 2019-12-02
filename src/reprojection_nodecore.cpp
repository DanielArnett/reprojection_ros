#include "reprojection_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    imageSubscriber_ = it_.subscribe("image_in" , 1, &SimpleRendererNode::imageCallback, this);

    int width, height;
    nh_.param<int>("width" , width , 640);
    nh_.param<int>("height", height, 480);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");
    
    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );
    
    float correction1, correction2, correction3, correction4, cropTop;
    float cropBottom, cropLeft, cropRight, xCenter, yCenter;
    float roll, yaw, x, y, z, fovIn, fovOut;
    int inputProjection, outputProjection, gridLines;
    float pitch;
    nh_.param<float>("correction1", correction1, 1.0);
    nh_.param<float>("correction2", correction2, 1.0);
    nh_.param<float>("correction3", correction3, 1.0);
    nh_.param<float>("correction4", correction4, 1.0);
    nh_.param<float>("cropTop", cropTop, 1.0);
    nh_.param<float>("cropBottom", cropBottom, 1.0);
    nh_.param<float>("cropLeft", cropLeft, 1.0);
    nh_.param<float>("cropRight", cropRight, 1.0);
    nh_.param<float>("xCenter", xCenter, 1.0);
    nh_.param<float>("yCenter", yCenter, 1.0);
    nh_.param<float>("pitch", pitch, 1.0);
    nh_.param<float>("roll", roll, 0.0);
    nh_.param<float>("yaw", yaw, 0.0);
    nh_.param<float>("fovIn", fovIn, 1.0);
    nh_.param<float>("fovOut", fovOut, 1.0);
    nh_.param<float>("x", x, 1.0);
    nh_.param<float>("y", y, 1.0);
    nh_.param<float>("z", z, 1.0);
    nh_.param<int>("inputProjection", inputProjection, 0);
    nh_.param<int>("outputProjection", outputProjection, 0);
    nh_.param<int>("gridLines", gridLines, 0);
    
    renderer_->uniform("correction1", correction1);
    renderer_->uniform("correction2", correction2);
    renderer_->uniform("correction3", correction3);
    renderer_->uniform("correction4", correction4);
    renderer_->uniform("cropTop", cropTop);
    renderer_->uniform("cropBottom", cropBottom);
    renderer_->uniform("cropLeft", cropLeft);
    renderer_->uniform("cropRight", cropRight);
    renderer_->uniform("xCenter", xCenter);
    renderer_->uniform("yCenter", yCenter);
    renderer_->uniform("pitch", pitch);
    renderer_->uniform("roll", roll);
    renderer_->uniform("yaw", yaw);
    renderer_->uniform("fovIn", fovIn);
    renderer_->uniform("fovOut", fovOut);
    renderer_->uniform("x", x);
    renderer_->uniform("y", y);
    renderer_->uniform("z", z);
    renderer_->uniform("inputProjection", inputProjection);
    renderer_->uniform("outputProjection", outputProjection);
    renderer_->uniform("gridLines", gridLines);

    output_.create(height, width, CV_8UC3);
}

void SimpleRendererNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }

    auto start = std::chrono::system_clock::now();

    const auto& image = cv_ptr->image;
    renderer_->render(output_, image);

    ROS_DEBUG_STREAM(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
    );

    //Publish
    cv_bridge::CvImage outImage;;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void SimpleRendererNode::run()
{
    ros::spin();
}
