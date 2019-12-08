#include "dual_fisheye_blending_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

void SimpleRendererNode::reconfigure_callback(reprojection::dual_fisheye_blendingConfig &config, uint32_t level) {
    //ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
    //        config.int_param, config.double_param, 
    //        config.str_param.c_str(), 
    //        config.bool_param?"True":"False", 
    //        config.size);
    //ROS_INFO("Reconfigure Request: %f", config.croppedWidth);
    left_recon_.blendFront	     = config.left_blendFront; 
    left_recon_.blendBack        = config.left_blendBack; 
    left_recon_.linearBlend      = config.left_linearBlend;
    left_recon_.correction1      = config.left_correction1;
    left_recon_.correction2      = config.left_correction2;
    left_recon_.correction3      = config.left_correction3;
    left_recon_.correction4      = config.left_correction4;
    left_recon_.croppedWidth     = config.left_croppedWidth;
    left_recon_.croppedHeight    = config.left_croppedHeight;
    left_recon_.xCenter          = config.left_xCenter;
    left_recon_.yCenter          = config.left_yCenter;
    left_recon_.pitch            = config.left_pitch;
    left_recon_.roll             = config.left_roll;
    left_recon_.yaw              = config.left_yaw;
    left_recon_.fovIn            = config.left_fovIn;
    left_recon_.fovOut           = config.left_fovOut;
    left_recon_.x                = config.left_x;
    left_recon_.y                = config.left_y;
    left_recon_.z                = config.left_z;
    left_recon_.inputProjection  = config.left_inputProjection;
    left_recon_.outputProjection = config.left_outputProjection;
    left_recon_.gridLines        = config.left_gridLines;
    left_recon_.blendImages      = config.left_blendImages;

    right_recon_.blendFront	      = config.right_blendFront; 
    right_recon_.blendBack        = config.right_blendBack; 
    right_recon_.linearBlend      = config.right_linearBlend;
    right_recon_.correction1      = config.right_correction1;
    right_recon_.correction2      = config.right_correction2;
    right_recon_.correction3      = config.right_correction3;
    right_recon_.correction4      = config.right_correction4;
    right_recon_.croppedWidth     = config.right_croppedWidth;
    right_recon_.croppedHeight    = config.right_croppedHeight;
    right_recon_.xCenter          = config.right_xCenter;
    right_recon_.yCenter          = config.right_yCenter;
    right_recon_.pitch            = config.right_pitch;
    right_recon_.roll             = config.right_roll;
    right_recon_.yaw              = config.right_yaw;
    right_recon_.fovIn            = config.right_fovIn;
    right_recon_.fovOut           = config.right_fovOut;
    right_recon_.x                = config.right_x;
    right_recon_.y                = config.right_y;
    right_recon_.z                = config.right_z;
    right_recon_.inputProjection  = config.right_inputProjection;
    right_recon_.outputProjection = config.right_outputProjection;
    right_recon_.gridLines        = config.right_gridLines;
    right_recon_.blendImages      = config.right_blendImages;

    rotated_recon_.blendFront       = config.rotated_blendFront; 
    rotated_recon_.blendBack        = config.rotated_blendBack; 
    rotated_recon_.linearBlend      = config.rotated_linearBlend;
    rotated_recon_.correction1      = config.rotated_correction1;
    rotated_recon_.correction2      = config.rotated_correction2;
    rotated_recon_.correction3      = config.rotated_correction3;
    rotated_recon_.correction4      = config.rotated_correction4;
    rotated_recon_.croppedWidth     = config.rotated_croppedWidth;
    rotated_recon_.croppedHeight    = config.rotated_croppedHeight;
    rotated_recon_.xCenter          = config.rotated_xCenter;
    rotated_recon_.yCenter          = config.rotated_yCenter;
    rotated_recon_.pitch            = config.rotated_pitch;
    rotated_recon_.roll             = config.rotated_roll;
    rotated_recon_.yaw              = config.rotated_yaw;
    rotated_recon_.fovIn            = config.rotated_fovIn;
    rotated_recon_.fovOut           = config.rotated_fovOut;
    rotated_recon_.x                = config.rotated_x;
    rotated_recon_.y                = config.rotated_y;
    rotated_recon_.z                = config.rotated_z;
    rotated_recon_.inputProjection  = config.rotated_inputProjection;
    rotated_recon_.outputProjection = config.rotated_outputProjection;
    rotated_recon_.gridLines        = config.rotated_gridLines;
    rotated_recon_.blendImages      = config.rotated_blendImages;
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
    dynamic_reconfigure::Server<reprojection::dual_fisheye_blendingConfig> server;
    dynamic_reconfigure::Server<reprojection::dual_fisheye_blendingConfig>::CallbackType f;
    f = boost::bind(boost::mem_fn(&SimpleRendererNode::reconfigure_callback), this, _1, _2);
    server.setCallback(f);
    ros::spin();
}
