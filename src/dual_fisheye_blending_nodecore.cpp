#include "dual_fisheye_blending_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

void SimpleRendererNode::reconfigure(reprojection::reprojectionConfig &config, re_config& this_re_config) {
    this_re_config.blendFront	    = config.blendFront; 
    this_re_config.blendBack        = config.blendBack; 
    this_re_config.linearBlend      = config.linearBlend;
    this_re_config.correction1      = config.correction1;
    this_re_config.correction2      = config.correction2;
    this_re_config.correction3      = config.correction3;
    this_re_config.correction4      = config.correction4;
    this_re_config.croppedWidth     = config.croppedWidth;
    this_re_config.croppedHeight    = config.croppedHeight;
    this_re_config.xCenter          = config.xCenter;
    this_re_config.yCenter          = config.yCenter;
    this_re_config.pitch            = config.pitch;
    this_re_config.roll             = config.roll;
    this_re_config.yaw              = config.yaw;
    this_re_config.fovIn            = config.fovIn;
    this_re_config.fovOut           = config.fovOut;
    this_re_config.x                = config.x;
    this_re_config.y                = config.y;
    this_re_config.z                = config.z;
    this_re_config.inputProjection  = config.inputProjection;
    this_re_config.outputProjection = config.outputProjection;
    this_re_config.gridLines        = config.gridLines;
    this_re_config.blendImages      = config.blendImages;
}
void SimpleRendererNode::reconfigure_left_callback(reprojection::reprojectionConfig &config, uint32_t level) {
    //ROS_INFO("Reconfigure Request: %f", config.croppedWidth);
    reconfigure(config, left_recon_);
}

void SimpleRendererNode::reconfigure_right_callback(reprojection::reprojectionConfig &config, uint32_t level) {
    reconfigure(config, right_recon_);
}

void SimpleRendererNode::reconfigure_right_rotated_callback(reprojection::reprojectionConfig &config, uint32_t level) {
    reconfigure(config, rotated_recon_);
}

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    std::string image_in, image_out;
    nh_.param<std::string>("image_in", image_in, "image_in");
    nh_.param<std::string>("image_out", image_out, "image_out");
    imagePublisher_  = it_.advertise(image_out, 1);
    imageSubscriber_ = it_.subscribe(image_in , 1, &SimpleRendererNode::imageCallback, this);

    nh_.param<int>("width" , width_ , 640);
    nh_.param<int>("height", height_, 480);

    nh_.param<std::string>("vertex_shader"  , vertexShader_, "");
    nh_.param<std::string>("reprojection_fragment_shader", reprojectionFragmentShader_, "");

    left_renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width_, height_, 
        vertexShader_, reprojectionFragmentShader_
    );


    right_renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width_, height_, 
        vertexShader_, reprojectionFragmentShader_
    );

    right_rotated_renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width_, height_, 
        vertexShader_, reprojectionFragmentShader_
    );


    left_output_.create(height_, width_, CV_8UC4);
    right_output_.create(height_, width_, CV_8UC4);
    blended_output_.create(height_, width_, CV_8UC4);
}
void SimpleRendererNode::setUniforms(std::unique_ptr<cgs::SimpleRenderer>& renderer, re_config& config) 
{
    renderer->uniform("blendFront", 		config.blendFront);
    renderer->uniform("blendBack", 		    config.blendBack);
    renderer->uniform("linearBlend", 	    config.linearBlend);
    renderer->uniform("correction1", 		config.correction1);
    renderer->uniform("correction2", 		config.correction2);
    renderer->uniform("correction3", 		config.correction3);
    renderer->uniform("correction4", 		config.correction4);
    renderer->uniform("croppedWidth", 		config.croppedWidth);
    renderer->uniform("croppedHeight", 	    config.croppedHeight);
    renderer->uniform("xCenter", 		    config.xCenter);
    renderer->uniform("yCenter", 		    config.yCenter);
    renderer->uniform("pitch", 		        config.pitch);
    renderer->uniform("roll", 			    config.roll);
    renderer->uniform("yaw", 			    config.yaw);
    renderer->uniform("fovIn", 		        config.fovIn);
    renderer->uniform("fovOut", 		    config.fovOut);
    renderer->uniform("x", 			        config.x);
    renderer->uniform("y", 			        config.y);
    renderer->uniform("z", 			        config.z);
    renderer->uniform("inputProjection", 	config.inputProjection);
    renderer->uniform("outputProjection", 	config.outputProjection);
    renderer->uniform("gridLines", 		    config.gridLines);
    renderer->uniform("blendImages", 		config.blendImages);
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

    setUniforms(left_renderer_, left_recon_);
    left_renderer_->render(left_output_, image);

    setUniforms(right_renderer_, right_recon_);
    right_renderer_->render(right_output_, image);
    
    setUniforms(right_rotated_renderer_, rotated_recon_);
    right_rotated_renderer_->render(blended_output_, right_output_, left_output_);

    ROS_DEBUG_STREAM(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
    );

    //Publish
    cv_bridge::CvImage outImage;
    outImage.header = cv_ptr->header;
    outImage.encoding = cv_ptr->encoding;
    outImage.image = blended_output_;
    imagePublisher_.publish(outImage.toImageMsg());
}

void SimpleRendererNode::run()
{
    dynamic_reconfigure::Server<reprojection::reprojectionConfig> left_server(ros::NodeHandle("left_reprojection"));
    dynamic_reconfigure::Server<reprojection::reprojectionConfig>::CallbackType left_f;
    left_f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_left_callback), this, _1, _2);
    left_server.setCallback(left_f);

    dynamic_reconfigure::Server<reprojection::reprojectionConfig> right_server(ros::NodeHandle("right_reprojection"));
    dynamic_reconfigure::Server<reprojection::reprojectionConfig>::CallbackType right_f;
    right_f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_right_callback), this, _1, _2);
    right_server.setCallback(right_f);

    dynamic_reconfigure::Server<reprojection::reprojectionConfig> right_rotated_server(ros::NodeHandle("right_rotated_reprojection"));
    dynamic_reconfigure::Server<reprojection::reprojectionConfig>::CallbackType right_rotated_f;
    right_rotated_f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_right_rotated_callback), this, _1, _2);
    right_rotated_server.setCallback(right_rotated_f);

    ros::spin();
}
