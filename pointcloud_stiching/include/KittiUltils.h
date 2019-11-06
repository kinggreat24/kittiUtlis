#ifndef KITTI_ULTILS_H
#define KITTI_ULTILS_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <thread>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <time.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
//Eigen
#include <eigen3/Eigen/Dense>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace kitti_utils{

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloud;

class KittiUltils{
public:
    KittiUltils(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~KittiUltils();
    void LoadFileNames(const std::string &strSequence, 
                        std::vector<std::string> &vstrImageLeftFilenames, 
                        std::vector<std::string> &vstrImageRightFilenames, 
                        std::vector<std::string> &vstrLidarFilenames,
                        std::vector<Eigen::Matrix<double,3,4> >& vPoses,
                        std::vector<double> &vTimestamps);
    void PublishTopics(const ros::TimerEvent &e);
protected:
    int ReadLidarFile(const std::string& lidar_file,pcl::PointCloud<PointType>::Ptr outpointcloud, bool isBinary);
    int ReadImageFile(const std::string& image, const int left);
    int ReadGroundTruthPose(const int t);
    int PointCloudStiching(const Eigen::Matrix<double,3,4>& pose, const pcl::PointCloud<PointType>::Ptr pointcloud);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // All sensor file names
    std::vector<std::string> image_left_files_;
    std::vector<std::string> image_right_files_;
    std::vector<std::string> lidar_files_;
    std::vector<Eigen::Matrix<double,3,4> >poses_;
    std::vector<double> timestamps_;

    // Image publisher
    image_transport::ImageTransport image_trasnport_;
    image_transport::Publisher image_left_pub_;
    image_transport::Publisher image_right_pub_;

    // Lidar publisher
    ros::Publisher current_laser_cloud_pub_/*, stiching_laser_cloud_pub_*/;
    pcl::PointCloud<PointType>::Ptr current_point_cloud_ptr_;
    // pcl::PointCloud<PointType>::Ptr stiching_point_cloud_ptr_;

    // Pose publisher
    // ros::Publisher pose_gt_pub_;

    // Ground truth odom publisher
    ros::Publisher odom_gt_pub_;

    // Ground truth path publisher
    ros::Publisher path_gt_pub_;

    // ROS message
    sensor_msgs::ImagePtr        imagePtr_left_;
    sensor_msgs::ImagePtr        imagePtr_right_;
    sensor_msgs::PointCloud2     current_frame_point_cloud_;
    sensor_msgs::PointCloud2     stiching_point_cloud_;
    nav_msgs::Odometry           odom_ground_truth_;
    nav_msgs::Path               path_ground_truth_;

    /**************         Parameter        **************/
    // Senosr frame id
    std::string image_left_frame_id_;
    std::string image_right_frame_id_;
    std::string pointcloud_frame_id_;
    
    bool isBGR_;
    bool export_rosbag_;
    std::string dataset_folder_, sequence_number_, output_bag_file_;
    double frequency_;
    ros::Duration publish_interval_;
    ros::Timer timer_;

    int count_;
    int nImages_;
    ros::Time currentTime_;
    Eigen::Matrix3d camera2baselink_matrix_;
    Eigen::Quaterniond camera2baselink_quaternion_;
    
    //tf
    tf::TransformBroadcaster odometry_broadcaster_;
};


}//End of namespace kitti_utils

#endif//KITTI_ULTILS_H