#include "reprojection_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

void SimpleRendererNode::reconfigure_callback(reprojection::reprojectionConfig &config, uint32_t level) {
    //ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
    //        config.int_param, config.double_param, 
    //        config.str_param.c_str(), 
    //        config.bool_param?"True":"False", 
    //        config.size);
    //ROS_INFO("Reconfigure Request: %f", config.croppedWidth);
  
    blendFront	     = config.blendFront; 
    blendBack        = config.blendBack;  
    blendImages      = config.blendImages; 
    linearBlend      = config.linearBlend;
    correction1      = config.correction1;
    correction2      = config.correction2;
    correction3      = config.correction3;
    correction4      = config.correction4;
    croppedWidth     = config.croppedWidth;
    croppedHeight    = config.croppedHeight;
    xCenter          = config.xCenter;
    yCenter          = config.yCenter;
    pitch            = config.pitch;
    roll             = config.roll;
    yaw              = config.yaw;
    fovIn            = config.fovIn;
    fovOut           = config.fovOut;
    x                = config.x;
    y                = config.y;
    z                = config.z;
    inputProjection  = config.inputProjection;
    outputProjection = config.outputProjection;
    gridLines        = config.gridLines;

    renderer_->uniform("blendFront", 		blendFront);
    renderer_->uniform("blendBack", 		blendBack);
    renderer_->uniform("blendImages", 		blendImages);
    renderer_->uniform("linearBlend", 		linearBlend);
    renderer_->uniform("correction1", 		correction1);
    renderer_->uniform("correction2", 		correction2);
    renderer_->uniform("correction3", 		correction3);
    renderer_->uniform("correction4", 		correction4);
    renderer_->uniform("croppedWidth", 		croppedWidth);
    renderer_->uniform("croppedHeight", 	croppedHeight);
    renderer_->uniform("xCenter", 		xCenter);
    renderer_->uniform("yCenter", 		yCenter);
    renderer_->uniform("pitch", 		pitch);
    renderer_->uniform("roll", 			roll);
    renderer_->uniform("yaw", 			yaw);
    renderer_->uniform("fovIn", 		fovIn);
    renderer_->uniform("fovOut", 		fovOut);
    renderer_->uniform("x", 			x);
    renderer_->uniform("y", 			y);
    renderer_->uniform("z", 			z);
    renderer_->uniform("inputProjection", 	inputProjection);
    renderer_->uniform("outputProjection", 	outputProjection);
    renderer_->uniform("gridLines", 		gridLines);
}


SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    std::string image_in, image_out;
    nh_.param<std::string>("image_in", image_in, "image_in");
    nh_.param<std::string>("image_out", image_out, "image_out");
    imagePublisher_  = it_.advertise(image_out, 1);
    imageSubscriber_ = it_.subscribe(image_in , 1, &SimpleRendererNode::imageCallback, this);

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
    
    output_.create(height, width, CV_8UC4);
}

void SimpleRendererNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
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
    cv_bridge::CvImage outImage;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void SimpleRendererNode::run()
{
    dynamic_reconfigure::Server<reprojection::reprojectionConfig> server;
    dynamic_reconfigure::Server<reprojection::reprojectionConfig>::CallbackType f;
    //f = boost::bind(reconfigure_callback, _1, _2);
    f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_callback), this, _1, _2);

    server.setCallback(f);
    ros::spin();
}
