#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <cmath>

#include "asv_ctrl_sb_mpc/asv_ctrl_sb_mpc.h"

const float PI = 3.1415927;
static const double DEG2RAD = PI/180.0f;
static const double RAD2DEG = 180.0f/PI;

simulationBasedMpc::simulationBasedMpc()
{
	asv_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
	asv_twist_ = Eigen::Vector3d(0.0, 0.0, 0.0);
};

simulationBasedMpc::~simulationBasedMpc()
{
};

void simulationBasedMpc::initialize(std::vector<asv_msgs::State> *obstacles,
						nav_msgs::OccupancyGrid *map)
{
	ROS_INFO("Initializing sb-mpc node...");
	
	Chi_ca_last = 0;   	// Keep nominal course
	P_ca_last = 1;		// Keep nominal speed
	
	
	/// @todo Remove local_map_! Only used for debugging purposes...
	local_map_.header.frame_id ="map";
	local_map_.info.resolution = 0.78;
	local_map_.info.width  = 1362;
	local_map_.info.height = 942;
	local_map_.info.origin.position.x = -(float)496;
	local_map_.info.origin.position.y = -(fhloat)560;

	local_map_.data.resize(local_map_.info.width*local_map_.info.height);
	ros::NodeHandle n;
	lm_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);
	obstacles_ = obstacles;
	map_ = map;
	
	ROS_INFO("Initialization complete");
}

void simulationBasedMpc::update()
{
	// Not sure if I need this...
};


void simulationBasedMpc::updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d)
{
  double yaw = tf::getYaw(msg->pose.pose.orientation);
  asv_pose_[0] = msg->pose.pose.position.x;
  asv_pose_[1] = msg->pose.pose.position.y;
  asv_pose_[2] = yaw;
  asv_twist_[0] = msg->twist.twist.linear.x;
  asv_twist_[1] = msg->twist.twist.linear.y;
  asv_twist_[2] = msg->twist.twist.angular.z;

  u_d_ = u_d;
  psi_d_ = psi_d;
};

void simulationBasedMpc::getBestControlInput(double &u_best, double &psi_best)
{
	// Setting the adjusted ctrl inputs to the unadjusted ctrl inputs
	// change this so that the first two offsets are correct
	u_best = u_d_;
	psi_best = psi_d_; 
	ROS_INFO("u_ca set to: %0.2f, psi_ca set to: %0.2f", u_d_, psi_d_);
	
};


