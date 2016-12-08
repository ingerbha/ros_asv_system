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


const float PI = M_PI;//3.1415927;
static const double DEG2RAD = PI/180.0f;
static const double RAD2DEG = 180.0f/PI;

// Utils
void rot2d(double yaw, Eigen::Vector2d &res);

simulationBasedMpc::simulationBasedMpc() : 	T_(30), //30
											DT_(0.05),
											P_(1),
											Q_(4.0),
											D_CLOSE_(120.0), //120
											D_SAFE_(40.0), //40
											K_COLL_(0.2),
											PHI_AH_(30.0),
											PHI_OT_(68.5),
											PHI_HO_(22.5),
											PHI_CR_(30), //30
											KAPPA_(200), //200
											K_P_(70), //70
											K_CHI_(120) //120
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

void simulationBasedMpc::getBestControlOffset(double &u_d_best, double &psi_d_best)
{
	
	std::vector<asv_msgs::State>::iterator it; // Obstacles iterator
	
	for (it = obstacles_->begin(); it != obstacles_->end(); ++it){
		obstacle *obst = new obstacle(it->x,it->y,it->u,it->v, it->psi, it->header.radius, T_, DT_);
		obstacles_vect.push_back(obst);
	}
	
	//DEBUG
//	obstacle *obst = new obstacle(0.0,200.0,2.1,0.0, -1.5708, T_, DT_);
//	obstacles_vect.push_back(obst);


	double u_os = 1;
	double psi_os = 0;
	double cost = INFINITY; 
	double cost_i;

//	ROS_DEBUG_STREAM_NAMED("Testing","Asv yaw: " << asv_pose_[2]*RAD2DEG << "\tObst yaw: " << obstacles_vect[0]->psi_*RAD2DEG);

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

	u_d_best = u_os;
	psi_d_best = psi_os;
	
	ROS_INFO("u_os: %0.2f      psi_os: %0.2f", u_os, psi_os*RAD2DEG);
};


double simulationBasedMpc::costFnc(double P_ca, double Chi_ca, int k)
{ 
	double combined_radius = asv->radius + obstacles_vect[k]->radius_;
	double d_safe = D_SAFE_ + combined_radius;
	double d_close = D_CLOSE_ + combined_radius;
	double dist; 
	Eigen::Vector2d d;
	Eigen::Vector2d los;
	double phi , psi_o, psi_rel, R, C, k_coll;
	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
	double cost = 0;
	bool mu, OT, SB, HO, CR;
	


	//DEBUG:
	bool mu_1 = false;
	bool mu_2 = false;
	bool mu_3 = false;

	Eigen::Vector2d v_o;
	Eigen::Vector2d v_s;

	double t = 0;
	double t0 = 0;

	for (int i = 0; i < n_samp-1; i++){

		t += DT_;
		
		d(0) = obstacles_vect[k]->x_[i] - asv->x[i];
		d(1) = obstacles_vect[k]->y_[i] - asv->y[i];
		dist = d.norm();

		v_o(0) = obstacles_vect[k]->u_[i];
		v_o(1) = obstacles_vect[k]->v_[i];
		rot2d(obstacles_vect[k]->psi_,v_o);

		v_s(0) = asv->u[i];
		v_s(1) = asv->v[i];
		rot2d(asv->psi[i],v_s);

		psi_o = obstacles_vect[k]->psi_;
		while(psi_o <= -M_PI) phi += 2*M_PI;
		while (psi_o > M_PI) phi -= 2*M_PI;

		R = 0;
		C = 0;
		mu = 0;

		if (dist < d_close){
			if (dist < d_safe){
				R = (1/pow(fabs(t-t0),P_))*pow(d_safe/dist,Q_);
				k_coll = K_COLL_*combined_radius;
				C = k_coll*pow((v_s-v_o).norm(),2);
			}
			los = d/dist;
			phi = atan2(d(1),d(0)) - asv->psi[i];
			while(phi <= -PI) phi += 2*PI;
			while (phi > PI) phi -= 2*PI;

			psi_rel = psi_o - asv->psi[i];
			while(psi_rel < -M_PI) psi_rel += 2*M_PI;
			while(psi_rel > M_PI) psi_rel -= 2*M_PI;


			// Overtaken by obstacle
			OT = v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm()  // ca samme retning
					&& v_s.norm() < v_o.norm();								// obst kjører raskere
			// Obstacle on starboard side
			SB = phi < 0;
			// Obstacle Head-on
			HO = v_o.norm() > 0.05
					&& v_s.dot(v_o) < -cos(PHI_HO_*DEG2RAD)*v_s.norm()*v_o.norm()   // hastigheter i motsatt retn
					&& v_s.dot(los) > cos(PHI_AH_*DEG2RAD)*v_s.norm();			 	// obst forran
			// Crossing situation
			CR = v_s.dot(v_o) < cos(PHI_CR_*DEG2RAD)*v_s.norm()*v_o.norm()			//
					&& ((SB && psi_rel > 0 && psi_rel < 150*DEG2RAD)
						|| (!SB && psi_rel < 0 && psi_rel > -150*DEG2RAD));		// obstacle heading towards ship (from either side..)


			mu = ( SB && HO ) || (SB && CR); //&& OT);

			//DEBUG
			if (SB) mu_1 = true;
			if (CR) mu_2 = true;
			if (OT) mu_3 = true;
		}

		H0 = C*R + KAPPA_*mu;

		// Maximizing the cost wrt. t
		if (H0 > H1){
			H1 = H0;
		}
	}

	H2 = K_P_*(1-P_ca) + K_CHI_*pow(Chi_ca,2) + Delta_P(P_ca) + Delta_Chi(Chi_ca);
	cost =  H1 + H2;

	// Print H1 and H2 for P==X
//	ROS_DEBUG_COND_NAMED(P_ca == 0.5,"Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
//	ROS_DEBUG_COND_NAMED(P_ca == 1 , "Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
	// Print H1 and H2 for all P
//	ROS_DEBUG_NAMED("Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
	// Print mu_1 and mu_2
//	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","Chi: " << Chi_ca*RAD2DEG << "  \tSB " << mu_1  << " CR " << mu_2 << " OT " << mu_3);
//	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","psi_o: "<<psi_o*RAD2DEG<<"\tpsi_s: "<<psi_s*RAD2DEG);

	return cost;
}

double simulationBasedMpc::Delta_P(double P_ca){
	
	return 40*std::abs(P_ca_last_ - P_ca); // 40-44
}

double simulationBasedMpc::Delta_Chi(double Chi_ca){
	double dChi = Chi_ca - Chi_ca_last_;
	if (dChi > 0){
		return 5 + 35*pow(dChi,2); //
	}else if (dChi < 0){
		return 30*pow(dChi,2); //
	}else{
		return 0;
	}
}


// Utils
void rot2d(double yaw, Eigen::Vector2d &res){
	Eigen::Matrix2d R;
	R << cos(yaw), -sin(yaw),
		 sin(yaw), cos(yaw);
	res = R*res;
}







