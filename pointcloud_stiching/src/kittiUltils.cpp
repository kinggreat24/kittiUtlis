#include "KittiUltils.h"
using namespace std;

namespace kitti_utils{
KittiUltils::KittiUltils(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , image_left_frame_id_("image_left")
    , image_right_frame_id_("image_right")
    , pointcloud_frame_id_("laser_link")
    , export_rosbag_(false)
    , dataset_folder_("")
    , sequence_number_("00")
    , output_bag_file_("kitti.bag")
    , isBGR_(false)
    , frequency_(10)
    , count_(0)
    , nImages_(0)
    , image_trasnport_(image_transport::ImageTransport(nh_))
{
    // Get ROS parameter
    nh_private_.param("image_left_frame_id", image_left_frame_id_, image_left_frame_id_);
    nh_private_.param("image_right_frame_id", image_right_frame_id_, image_right_frame_id_);
    nh_private_.param("pointcloud_frame_id", pointcloud_frame_id_, pointcloud_frame_id_);
    nh_private_.param("export_rosbag", export_rosbag_, export_rosbag_);
    nh_private_.param("dataset_folder", dataset_folder_, dataset_folder_);
    nh_private_.param("sequence_number", sequence_number_, sequence_number_);
    nh_private_.param("output_bag_file", output_bag_file_, output_bag_file_);
    nh_private_.param("publish_frequency", frequency_, frequency_);

    // Publisher of image
    // image_trasnport_ = new image_transport::ImageTransport(nh_);
    image_left_pub_ = image_trasnport_.advertise("/image_left", 1);
    image_right_pub_ = image_trasnport_.advertise("/image_right", 1);

    // Publisher of PointCloud2
    current_laser_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("current_frame_pointcloud", 1);
    // stiching_laser_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("stiching_pointcloud", 1);

    // Publisher of Pose
    // pose_gt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_gt",1);

    // Publisher of Odom and Path that are groudtruth
    odom_gt_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_gt", 1);
    path_gt_pub_ = nh_.advertise<nav_msgs::Path>("path_gt", 1);

    // Load file names
    LoadFileNames(dataset_folder_,image_left_files_,image_right_files_,lidar_files_,poses_,timestamps_);
    ROS_INFO("Totally read: %d files",(int)timestamps_.size());
    //
    publish_interval_ = ros::Duration(1.0 / max(frequency_, 1.0));
    timer_ = nh_private_.createTimer(publish_interval_, &KittiUltils::PublishTopics, this,false);
    ROS_INFO("KittiUltils construct over.");

    //transform of Camera2BaseLink
    camera2baselink_matrix_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    camera2baselink_quaternion_ = Eigen::Quaterniond(camera2baselink_matrix_);
}

KittiUltils::~KittiUltils() {}

void KittiUltils::LoadFileNames(const std::string &strPathToSequence,
                                std::vector<std::string> &vstrImageLeftFilenames,
                                std::vector<std::string> &vstrImageRightFilenames,
                                std::vector<std::string> &vstrLidarFilenames,
                                std::vector<Eigen::Matrix<double, 3, 4>> &vPoses,
                                std::vector<double> &vTimestamps)
{
    // Read timestamps
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "sequences/" + sequence_number_ + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        std::getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    nImages_ = vTimestamps.size();

    // Read poses
    ifstream fPoses;
    std::string strPathGroundTruthOdomFile = strPathToSequence + "poses/" + sequence_number_ + ".txt";
    fPoses.open(strPathGroundTruthOdomFile.c_str());
    while (!fPoses.eof())
    {
        std::string line,s;
        std::getline(fPoses, line);
        if(line.empty())
            break;
        std::stringstream pose_stream(line);
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }
        vPoses.push_back(gt_pose);
    }

    if(vTimestamps.size() != vPoses.size())
    {
        std::cerr << "The pose number is not equal to images"<< std::endl;
        exit(EXIT_FAILURE);
    }

    // Read image files and lidar files
    string strPrefixLeft  = strPathToSequence + "sequences/" + sequence_number_ + "/image_0/";
    string strPrefixRight = strPathToSequence + "sequences/" + sequence_number_ + "/image_1/";
    string strPrefixLidar = strPathToSequence + "sequences/" + sequence_number_ + "/velodyne/";

    const int nTimes = vTimestamps.size();
    vstrImageLeftFilenames.resize(nTimes);
    vstrImageRightFilenames.resize(nTimes);
    vstrLidarFilenames.resize(nTimes);


    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeftFilenames[i]  = strPrefixLeft + ss.str() + ".png";
        vstrImageRightFilenames[i] = strPrefixRight + ss.str() + ".png";
        vstrLidarFilenames[i] = strPrefixLidar + ss.str() + ".bin";
    }
}


int KittiUltils::ReadLidarFile(const std::string &file, pcl::PointCloud<PointType>::Ptr outpointcloud, bool isBinary)
{
    outpointcloud->height = 1;
    if (isBinary)
    {
        // load point cloud
        std::fstream input(file.c_str(), std::ios::in | std::ios::binary);
        if (!input.good())
        {
            std::cerr << "Could not read file: " << file << std::endl;
            exit(EXIT_FAILURE);
        }
        //LOG(INFO)<<"Read: "<<_file<<std::endl;

        for (int i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI point;
            input.read((char *)&point.x, 3 * sizeof(float));
            input.read((char *)&point.intensity, sizeof(float));

            //remove all points behind image plane (approximation)
            /*if (point.x < mMinDepth)
                continue;*/
            outpointcloud->points.push_back(point);
        }
        outpointcloud->width = outpointcloud->points.size();
    }
    else
    {
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(file, *outpointcloud))
        {
            std::cerr << "Could not read file: " << file << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    return outpointcloud->points.size();
}


int KittiUltils::ReadImageFile(const std::string &file, const int left)
{
    // Read image from file
    cv::Mat image = cv::imread(file, CV_LOAD_IMAGE_UNCHANGED);
    if (image.empty())
    {
        cerr << endl
             << "Failed to load image at: " << file << endl;
        return -1;
    }

    // Left or right image to read
    if (left == 0)
    {
        std_msgs::Header header;
        header.frame_id = image_left_frame_id_;

        if (image.channels() == 1)
        {
            imagePtr_left_ = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        }
        else if (image.channels() == 3)
        {
            if (isBGR_)
                imagePtr_left_ = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
            else
                imagePtr_left_ = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();
        }
        else if (image.channels() == 4)
        {
            if (isBGR_)
                imagePtr_left_ = cv_bridge::CvImage(header, "bgra8", image).toImageMsg();
            else
                imagePtr_left_ = cv_bridge::CvImage(header, "rgba8", image).toImageMsg();
        }
    }
    else
    {
        std_msgs::Header header;
        header.frame_id = image_right_frame_id_;

        if (image.channels() == 1)
        {
            imagePtr_right_ = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        }
        else if (image.channels() == 3)
        {
            if (isBGR_)
                imagePtr_right_ = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
            else
                imagePtr_right_ = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();
        }
        else if (image.channels() == 4)
        {
            if (isBGR_)
                imagePtr_right_ = cv_bridge::CvImage(header, "bgra8", image).toImageMsg();
            else
                imagePtr_right_ = cv_bridge::CvImage(header, "rgba8", image).toImageMsg();
        }
    }
}


void KittiUltils::PublishTopics(const ros::TimerEvent &e)
{
    if(count_ >= nImages_)
        timer_.stop();
    
    // Read data from file  thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    current_point_cloud_ptr_.reset(new pcl::PointCloud<PointType>());
    thread left_image_thread(&KittiUltils::ReadImageFile,this,image_left_files_[count_],0);
    thread right_image_thread(&KittiUltils::ReadImageFile,this,image_right_files_[count_],1);
    thread lidar_thread(&KittiUltils::ReadLidarFile,this,lidar_files_[count_],current_point_cloud_ptr_,true);
    left_image_thread.join();
    right_image_thread.join();
    lidar_thread.join();

    currentTime_ = ros::Time::now();

    // Publish Images
    if(image_left_pub_.getNumSubscribers())
    {
        imagePtr_left_->header.stamp = currentTime_;
        image_left_pub_.publish(imagePtr_left_);
    }
        

    if(image_right_pub_.getNumSubscribers())
    {
        imagePtr_right_->header.stamp = currentTime_;
        image_right_pub_.publish(imagePtr_right_);
    }
        
    // Publish Current Lidar PointCloud
    if(current_laser_cloud_pub_.getNumSubscribers())
    {
        sensor_msgs::PointCloud2 current_pointcloud;
        pcl::toROSMsg(*(current_point_cloud_ptr_),current_pointcloud);
        current_pointcloud.header.frame_id = pointcloud_frame_id_;
        current_pointcloud.header.stamp = currentTime_;
        current_laser_cloud_pub_.publish(current_pointcloud);
    }

    // Publish Current pose
    if(/*odom_gt_pub_.getNumSubscribers()*/1)
    {   
        // Eigen::Quaterniond q_w_i(poses_[count_].topLeftCorner<3, 3>());
        // Eigen::Quaterniond q = camera2baselink_quaternion_ * q_w_i;
        // q.normalize();
        Eigen::Vector3d translation = camera2baselink_quaternion_ * poses_[count_].topRightCorner<3, 1>();

        Eigen::Matrix3d car_gt_orientation = poses_[count_].transpose().topLeftCorner<3, 3>();
        Eigen::Vector3d euler_angles = car_gt_orientation.eulerAngles( 1,0,2 );                               //YXZ顺序
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_angles[2], euler_angles[1], euler_angles[0]);

        odom_ground_truth_.header.stamp = currentTime_;
        odom_ground_truth_.header.frame_id = "odom";
        odom_ground_truth_.child_frame_id = "base_footprint";
        odom_ground_truth_.pose.pose.orientation.x = geoQuat.x;
        odom_ground_truth_.pose.pose.orientation.y = geoQuat.y;
        odom_ground_truth_.pose.pose.orientation.z = geoQuat.z;
        odom_ground_truth_.pose.pose.orientation.w = geoQuat.w;
        odom_ground_truth_.pose.pose.position.x = translation(0);
        odom_ground_truth_.pose.pose.position.y = translation(1);
        odom_ground_truth_.pose.pose.position.z = translation(2);
        odom_gt_pub_.publish(odom_ground_truth_);

        // Publish tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = currentTime_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = translation[0];
        odom_trans.transform.translation.y = translation[1];
        odom_trans.transform.translation.z = translation[2];
        odom_trans.transform.rotation = odom_ground_truth_.pose.pose.orientation;
        odometry_broadcaster_.sendTransform(odom_trans);

        // Add to path
        geometry_msgs::PoseStamped poseGT;
        poseGT.header.frame_id = "odom";
        poseGT.pose = odom_ground_truth_.pose.pose;
        path_ground_truth_.header.frame_id = odom_ground_truth_.header.frame_id;
        path_ground_truth_.poses.push_back(poseGT);
    }
    
    // Publish Path
    if(path_gt_pub_.getNumSubscribers())
    {
        path_gt_pub_.publish(path_ground_truth_);
    }

    count_++;
}


}//end of namespace kitti_utils