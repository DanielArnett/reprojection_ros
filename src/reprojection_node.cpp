#include "reprojection_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reprojection");

    SimpleRendererNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
