#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class TF_pointcloud
{
private:
  ros::NodeHandle private_nh;
  tf::TransformListener listener_;
  tf::StampedTransform transform;
  std_msgs::Header header;
  Eigen::Matrix4f eigen_transform;
  sensor_msgs::PointCloud2 output_pc2;
  ros::Subscriber sub;
  ros::Publisher pub;

public:
  TF_pointcloud()
  {
    sub = private_nh.subscribe ("gps_stamp_points", 1, &TF_pointcloud::cloud_cb, this);
    pub = private_nh.advertise<sensor_msgs::PointCloud2> ("transformed_points", 0);
  }
  void cloud_cb (const sensor_msgs::PointCloud2& input_pc2)
  {
    try{
      header.stamp = input_pc2.header.stamp;
	    listener_.waitForTransform("/map", "/velodyne", header.stamp, ros::Duration(3.0));
	    listener_.lookupTransform("/map", "/velodyne", header.stamp, transform);

	    pcl_ros::transformAsMatrix(transform, eigen_transform);
	    pcl_ros::transformPointCloud(eigen_transform, input_pc2, output_pc2);
      output_pc2.header.frame_id = "/map";
      output_pc2.header.stamp = input_pc2.header.stamp;

	    pub.publish(output_pc2);
    }
    catch(tf::LookupException){
      ROS_ERROR("Lookup_Exception");
    }
    catch(tf::ExtrapolationException){
      ROS_ERROR("Extrapolation_Exception");
    }
    catch(tf::InvalidArgument){
      ROS_ERROR("Invalid_Argument");
    }
    catch(tf::ConnectivityException){
      ROS_ERROR("Connectivity_Exception");
    }
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pc2_transform_node");
  TF_pointcloud TFObject;
  ros::spin();
}
