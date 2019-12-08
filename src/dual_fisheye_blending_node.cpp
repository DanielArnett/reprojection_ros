#include "dual_fisheye_blending_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_fisheye_blending");

    SimpleRendererNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
