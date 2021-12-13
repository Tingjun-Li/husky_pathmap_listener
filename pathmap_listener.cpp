#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <limits>
// #include <mutex>
// #include <thread>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

std::string file_name_kitti = "path_kitti";
std::string file_name_tum = "path_tum";
typedef std::numeric_limits< double > dbl;

void savePoseCallback(const geometry_msgs::Pose& pose, const long double timestamp);

void pathMapCallback(const nav_msgs::Path::ConstPtr& msg)
{
  geometry_msgs::Pose pose;
  pose = msg->poses.back().pose;
  geometry_msgs::Point position;
  position = pose.position;

  ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg->header.stamp.sec + (double)msg->header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp);
}

void savePoseCallback(const geometry_msgs::Pose& pose, const long double timestamp) {
    if (file_name_kitti.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");
        std::ofstream outfile(file_name_kitti,std::ofstream::out | std::ofstream::app );
        outfile << "1 0 0 "<< pose.position.x <<" 0 1 0 "<< pose.position.y <<" 0 0 1 "<< pose.position.z <<std::endl<<std::flush;
        outfile.close();
        // tum style
        std::ofstream tum_outfile(file_name_tum,std::ofstream::out | std::ofstream::app );
        // std::cout.precision(dbl::max_digits10);
        // std::cout << timestamp << std::endl;
        tum_outfile.precision(dbl::max_digits10);
        tum_outfile << timestamp << " "<< pose.position.x <<" "<< pose.position.y << " "<< pose.position.z << " "<< pose.orientation.x\
        <<" "<< pose.orientation.y <<" "<< pose.orientation.z <<" "<< pose.orientation.w <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  std::ofstream kitti_outfile(file_name_kitti);
  std::ofstream tum_outfile(file_name_tum);
  kitti_outfile.close();
  tum_outfile.close();

  ros::Subscriber sub = n.subscribe("zed_node/path_map", 1000, pathMapCallback);

  ros::spin();

  return 0;
}