#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include<iomanip>


//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>                     

//OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

class PointCloudStiching{
public:
    PointCloudStiching(const std::string& setting_file);
    ~PointCloudStiching();
protected:
    void onPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_ptr);
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudVoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const float x, const float y, const float z);
private:
    ros::Publisher stiching_cloud_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr stiching_pointcloud_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud2_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::PointCloud2> * tf_pointcloud_filter_;
    ros::NodeHandle nh_;
    std::string target_frame_;

    double m_pointcloudMinX,m_pointcloudMaxX,m_pointcloudMinY,m_pointcloudMaxY,m_pointcloudMinZ,m_pointcloudMaxZ;
    double m_leaf_x_,m_leaf_y_,m_leaf_z_;

    std::string pointcloud_save_dir_;
    int pointcloud_save_interval_;

    Eigen::Vector3f last_save_position_;              // Last pointcloud save position
    Eigen::Vector3f current_position_;                // Current pointcloud save position
    bool isGetFirstPosition_;
    int pointcloud_count_;                            // Pointcloud id
    pcl::PCDWriter pointcloud_writer_;
};


PointCloudStiching::PointCloudStiching(const std::string& setting_file)
    : nh_()
    , target_frame_("/map")
    , m_pointcloudMinX(-std::numeric_limits<double>::max())
    , m_pointcloudMaxX(std::numeric_limits<double>::max())
    , m_pointcloudMinY(-std::numeric_limits<double>::max())
    , m_pointcloudMaxY(std::numeric_limits<double>::max())
    , m_pointcloudMinZ(-std::numeric_limits<double>::max())
    , m_pointcloudMaxZ(std::numeric_limits<double>::max())
    , m_leaf_x_(0.5)
    , m_leaf_y_(0.5)
    , m_leaf_z_(0.5)
    , pointcloud_save_interval_(100)
    , pointcloud_save_dir_("")
    , isGetFirstPosition_(false)
    , pointcloud_count_(0)
{
    stiching_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // Read parameters
    cv::FileStorage fSettings(setting_file, cv::FileStorage::READ);
    // vocabulay_ = (std::string)fSettings["Vocabulary"];
    m_pointcloudMinX = fSettings["PCL_FILTER.PointcloudMinX"];
    m_pointcloudMaxX = fSettings["PCL_FILTER.PointcloudMaxX"];
    m_pointcloudMinY = fSettings["PCL_FILTER.PointcloudMinY"];
    m_pointcloudMaxY = fSettings["PCL_FILTER.PointcloudMaxY"];
    m_pointcloudMinZ = fSettings["PCL_FILTER.PointcloudMinZ"];
    m_pointcloudMaxZ = fSettings["PCL_FILTER.PointcloudMaxZ"];

    m_leaf_x_ = fSettings["PCL_VOXEL.LeafX"];
    m_leaf_y_ = fSettings["PCL_VOXEL.LeafY"];
    m_leaf_z_ = fSettings["PCL_VOXEL.LeafZ"];

    pointcloud_save_interval_ = fSettings["POINTCLOUD.SaveInterval"];
    pointcloud_save_dir_ = (std::string)fSettings["POINTCLOUD.SaveDIR"];

    // Subscibe pointcloud
    pointcloud2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"cloud_in",1);
    tf_pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud2_sub_, tf_, target_frame_, 1);
    tf_pointcloud_filter_->registerCallback( boost::bind(&PointCloudStiching::onPointCloudCallback, this, _1) );

    // Publisher
    stiching_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("stiching_pointcloud",1);

    // pcl pointcloud filter

}

PointCloudStiching::~PointCloudStiching(){}

void PointCloudStiching::onPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZI> pc; // input cloud for filtering
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try {
        tf_.lookupTransform(target_frame_, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    //std::cout<<"sensorToWorld: "<<std::endl<<sensorToWorld.matrix()<<std::endl;
    if(!isGetFirstPosition_)
    {
        last_save_position_ = sensorToWorld.block<3,1>(0,3);
        isGetFirstPosition_ = true;
    }    
    else
        current_position_ = sensorToWorld.block<3,1>(0,3);

    // set up filter for height range, also removes NANs:
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    *stiching_pointcloud_ += pc;

    stiching_pointcloud_ = pointCloudVoxelFilter(stiching_pointcloud_,m_leaf_x_,m_leaf_y_,m_leaf_z_);

    //Publish cloud
    if(stiching_cloud_pub_.getNumSubscribers())
    {
        sensor_msgs::PointCloud2 current_pointcloud;
        pcl::toROSMsg(*(stiching_pointcloud_),current_pointcloud);
        current_pointcloud.header.frame_id = target_frame_;
        current_pointcloud.header.stamp = ros::Time::now();
        stiching_cloud_pub_.publish(current_pointcloud);
    }

    // If the distance is beyond to the pointcloud_save_interval_, then save the pointcloud to the file
    float distance = (current_position_ - last_save_position_).norm();
    if(distance >= pointcloud_save_interval_)
    {
        // Save pointcloud
        std::stringstream ss;
        ss << setfill('0') << setw(6) << pointcloud_count_;
        pointcloud_writer_.write<pcl::PointXYZI>((pointcloud_save_dir_ + ss.str()+ ".pcd").c_str(), *stiching_pointcloud_); 
        ROS_INFO("Save pointcloud to file: %s",(pointcloud_save_dir_ + ss.str()+".pcd").c_str());
        stiching_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>() );

        last_save_position_ = current_position_;
        pointcloud_count_++;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudStiching::pointCloudVoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
        const float x, 
        const float y, 
        const float z)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZI>);//
 
    pcl::VoxelGrid<pcl::PointXYZI> voxelgrid; 
    voxelgrid.setInputCloud(cloud);                        //输入点云数据
    voxelgrid.setLeafSize(x, y, z);                        //AABB长宽高
    voxelgrid.filter(*cloud_after_voxelgrid);
    return cloud_after_voxelgrid;
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"pointcloud_stiching_node");
    ros::NodeHandle nh,nh_private("~");

    std::string setting_file("");
    nh_private.param("setting_file", setting_file, setting_file);

    if(setting_file.empty())
    {
        ROS_ERROR("The setting file is empty.");
        return -1;
    }

    PointCloudStiching pointcloudSticing(setting_file);

    ros::spin();

    return 0;
}