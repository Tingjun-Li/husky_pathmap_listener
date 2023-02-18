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
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>

std::string orb_slam3_file_name_tum = "orb_slam3_path_tum.txt";
std::string curly_file_name_tum = "curly_path_tum.txt";
typedef std::numeric_limits<double> dbl;

void savePoseCallback(const geometry_msgs::Pose &pose, const long double timestamp, const std::string &file_name_tum);

void Orb3PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
  geometry_msgs::Pose pose;
  pose = msg->poses.back().pose;
  geometry_msgs::Point position;
  position = pose.position;

  // ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg->header.stamp.sec + (double)msg->header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp, orb_slam3_file_name_tum);
}

void CurlyPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
  geometry_msgs::Pose pose;
  pose = msg->poses.back().pose;
  geometry_msgs::Point position;
  position = pose.position;

  // ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg->header.stamp.sec + (double)msg->header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp, curly_file_name_tum);
}

void cameraOdomCallback(const nav_msgs::Odometry &msg)
{
  geometry_msgs::Pose pose;
  pose = msg.pose.pose;
  geometry_msgs::Point position;
  position = pose.position;

  // ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg.header.stamp.sec + (double)msg.header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp, curly_file_name_tum);
}

void savePoseCallback(const geometry_msgs::Pose &pose, const long double timestamp, const std::string &file_name_tum)
{
  if (file_name_tum.size() > 0)
  {
    // ROS_INFO_STREAM("write new pose\n");
    // tum style
    std::ofstream tum_outfile(file_name_tum, std::ofstream::out | std::ofstream::app);
    // std::cout.precision(dbl::max_digits10);
    // std::cout << timestamp << std::endl;
    tum_outfile.precision(dbl::max_digits10);
    tum_outfile << timestamp << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << pose.orientation.x
                << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << std::endl
                << std::flush;

    tum_outfile.close();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  std::ofstream curly_tum_outfile(curly_file_name_tum);
  curly_tum_outfile.close();
  std::ofstream orb_slam3_tum_outfile(orb_slam3_file_name_tum);
  orb_slam3_tum_outfile.close();
  std::cout << "Ready to subscribe to path_map topic" << std::endl;
  ros::Subscriber sub_orb_slam3 = n.subscribe("ZED_ORBSLAM3/trajectory", 1000, Orb3PathCallback);
  ros::Subscriber sub_curly = n.subscribe("/robot/inekf_estimation/path", 1000, CurlyPathCallback);

  // ros::Subscriber sub = n.subscribe("zed_node/odom", 1000, cameraOdomCallback);

  ros::spin();

  return 0;
}