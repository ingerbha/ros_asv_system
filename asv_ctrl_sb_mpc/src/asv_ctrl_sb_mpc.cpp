#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <Eigen/Dense>

#include "asv_ctrl_sb_mpc/shipModel.h"
#include "asv_ctrl_sb_mpc/obstacle.h"
#include "asv_ctrl_sb_mpc/asv_ctrl_sb_mpc.h"

const float PI = 3.1415927;
static const double DEG2RAD = PI/180.0f;
static const double RAD2DEG = 180.0f/PI;

// Utils
void rot2d(double yaw, Eigen::Vector2d &res);

simulationBasedMpc::simulationBasedMpc() : 	T_(100),
											DT_(0.05),
											P_(1),
											Q_(4.0),
											D_CLOSE_(100.0),
											D_SAFE_(50.0),
											K_COLL_(0.01),
											PHI_AH_(45.0),
											PHI_OT_(68.5),
											PHI_HO_(22.5),
											PHI_CR_(68.5),
											KAPPA_(80),
											K_P_(400),
											K_CHI_(155) //155
{
	n_samp = floor(T_/DT_);
	asv_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
	asv_twist_ = Eigen::Vector3d(0.0, 0.0, 0.0);
};

simulationBasedMpc::~simulationBasedMpc()
{
};

void simulationBasedMpc::initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map)
{
	ROS_INFO("Initializing sb-mpc node...");
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	Chi_ca_last_ = 0;   	// Keep nominal course
	P_ca_last_ = 1;		    // Keep nominal speed
	
    double courseOffsets[] = {-90.0,-75.0,-60.0,-45.0,-30.0,-15.0,0.0,15.0,30.0,45.0,60.0,75.0,90.0};
    double sizeCO = sizeof(courseOffsets)/sizeof(courseOffsets[0]);
    for (int i = 0; i < sizeCO; i++){
    	courseOffsets[i] *= DEG2RAD;
    }
	Chi_ca_.assign(courseOffsets, courseOffsets + sizeof(courseOffsets)/sizeof(courseOffsets[0]));
	double speedOffsets[] = {-1,0,0.5,1};
	P_ca_.assign(speedOffsets, speedOffsets + sizeof(speedOffsets)/sizeof(speedOffsets[0]));

	// TODO: Set n_samp for shipModel and obstacle at the same place
	asv = new shipModel(T_,DT_);

	/// @todo Remove local_map_! Only used for debugging purposes...
	local_map_.header.frame_id ="map";
	local_map_.info.resolution = 0.78;
	local_map_.info.width  = 1362;
	local_map_.info.height = 942;
	local_map_.info.origin.position.x = -(float)496;
	local_map_.info.origin.position.y = -(float)560;

	local_map_.data.resize(local_map_.info.width*local_map_.info.height);
	ros::NodeHandle n;
	lm_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);
	obstacles_ = obstacles;
	map_ = map;
	
	ROS_INFO("Initialization complete");
}

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
	
	std::vector<asv_msgs::State>::iterator it; // Obstacles iterator
	
	for (it = obstacles_->begin(); it != obstacles_->end(); ++it){
		obstacle *obst = new obstacle(it->x,it->y,it->u,it->v, it->psi, T_, DT_);
		obstacles_vect.push_back(obst);
	}
	
	//DEBUG
	//obstacle *obst = new obstacle(0.0,200.0,2.1,0.0, -1.5708, T_, DT_);
	//obstacles_vect.push_back(obst);


	double u_os = 1;
	double psi_os = 0;
	double cost = INFINITY; 
	double cost_i;

	for (int i = 0; i < Chi_ca_.size(); i++){
		for (int j = 0; j < P_ca_.size(); j++){

			asv->eulersMethod(asv_pose_, asv_twist_, u_d_*P_ca_[j], psi_d_+ Chi_ca_[i]);

			for (int k = 0; k < obstacles_vect.size(); k++){

				cost_i = costFnc(P_ca_[j], Chi_ca_[i], k);
				
				if (cost_i < cost){
					cost = cost_i;
					u_os = P_ca_[j];
					psi_os = Chi_ca_[i];
				}
			}

			if (obstacles_vect.size() == 0){
				u_os = 1;
				psi_os = 0;
				cost = 0;
			}
		}
	}
	
	for (int k = 0; k < obstacles_vect.size(); k++){
		delete(obstacles_vect[k]);
	}
	obstacles_vect.clear();

	P_ca_last_ = u_os;
	Chi_ca_last_ = psi_os;

	// TODO: change something so that the first two offsets are correct
	u_best = u_d_*u_os;
	psi_best = psi_d_+ psi_os; 
	
	ROS_INFO("u_os: %0.2f      psi_os: %0.2f", u_os, psi_os*RAD2DEG);
};


double simulationBasedMpc::costFnc(double P_ca, double Chi_ca, int k)
{ 
	double d_safe = D_SAFE_; 	// TODO: Add term that takes into account the size of the ship
	double d_close = D_CLOSE_;  // TODO: Add term that takes into account the size of the ship
	double dist; 
	Eigen::Vector2d d;
	Eigen::Vector2d los;
	double phi, R, C, k_coll;
	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
	double cost = 0;
	bool mu, OT, SB, HO, CR;
	
	Eigen::Vector2d v_o;
	Eigen::Vector2d v_s;

	double t = 0;
	double t0 = 0;

	for (int i = 0; i < n_samp-1; i++){

		t += DT_;
		
		d(0) = obstacles_vect[k]->x[i] - asv->x[i];
		d(1) = obstacles_vect[k]->y[i] - asv->y[i];
		dist = d.norm();

		v_o(0) = obstacles_vect[k]->u[i];
		v_o(1) = obstacles_vect[k]->v[i];
		rot2d(obstacles_vect[k]->psi,v_o);

		v_s(0) = asv->u[i];
		v_s(1) = asv->v[i];
		rot2d(asv->psi[i],v_s);

		R = 0;
		C = 0;
		mu = 0;

		if (dist < d_close){
			if (dist < d_safe){
				R = (1/pow(std::abs(t-t0),P_))*pow(d_safe/dist,Q_);
				k_coll = K_COLL_*160; // TODO: Add term that takes into account the size of the ship
				C = k_coll*pow((v_s-v_o).norm(),2);
			}
			los = d/dist;
			phi = atan2(d(1),d(0)) - asv->psi[i];
			while(phi <= -PI) phi += 2*PI;
			while (phi > PI) phi -= 2*PI;

			OT = v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm()
					&& v_s.norm() < v_o.norm();
			SB = phi < 0;
			HO = v_o.norm() > 0.05
					&& v_s.dot(v_o) < -cos(PHI_HO_*DEG2RAD)*v_s.norm()*v_o.norm()
					&& v_s.dot(los) > cos(PHI_AH_*DEG2RAD)*v_s.norm();
			CR = v_s.dot(v_o) < cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm();
			
			mu = ( SB && HO ) || (SB && CR && !OT);
		}

		H0 = C*R + KAPPA_*mu;

		// Maximizing the cost wrt. t
		if (H0 > H1){
			H1 = H0;
		}

	}

	H2 = K_P_*(1-P_ca) + K_CHI_*pow(Chi_ca,2) + Delta_P(P_ca) + Delta_Chi(Chi_ca);
	cost =  H1 + H2;
	
	//ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","Chi : " << Chi_ca*RAD2DEG << "\tK_chi : " << K_CHI_*pow(Chi_ca,2) << "\tD_chi : " << Delta_Chi(Chi_ca)<<"\tH2 : " << H2);
	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","Chi : " << Chi_ca*RAD2DEG << "  \tP : " << P_ca << "\tH1: "<< H1 <<"    \tH2: " << H2);
	//ROS_DEBUG_STREAM_NAMED("Testing","Chi : " << Chi_ca*RAD2DEG << "  \tP : " << P_ca << "\tH1: "<< H1 <<"\tH2: " << H2);
	//ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1,"Testing","Chi : " << Chi_ca*RAD2DEG << "  \tP : " << P_ca << "\tH1: "<< H1 <<" \tphi " << phi*RAD2DEG << "\tpsi " << asv->psi[1999]*RAD2DEG);

	return cost;
}

double simulationBasedMpc::Delta_P(double P_ca){
	
	return 120*std::abs(P_ca_last_ - P_ca);
}

double simulationBasedMpc::Delta_Chi(double Chi_ca){
	double dChi = Chi_ca - Chi_ca_last_;
	if (dChi >= 0){
		return 30*fabs(dChi) + 40*pow(dChi,2);
	}else{
		return 20*fabs(dChi) + 40*pow(dChi,2);
	}
}


// Utils

void rot2d(double yaw, Eigen::Vector2d &res){
	Eigen::Matrix2d R;
	R << cos(yaw), -sin(yaw),
		 sin(yaw), cos(yaw);
	res = R*res;
}







