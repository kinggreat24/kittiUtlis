#include "KittiUltils.h"

using namespace kitti_utils;

int main(int argc, char** argv)
{
    ros::init(argc, argv,"kitti_ultils_node");
    ros::NodeHandle nh,nh_private("~");
    KittiUltils kittiUltils(nh,nh_private);
    ros::spin();
    return 0;
}