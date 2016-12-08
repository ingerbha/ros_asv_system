#include <ros/console.h>

#include <cmath>

#include "asv_ctrl_sb_mpc/obstacle.h"

	

obstacle::obstacle(double x_0, double y_0, double u_0, double v_0,  double psi_0, double radius, double T, double dt)
{
	T_ = T;
	dt_ = dt;
	n_samp_ = T_/dt_;
	
	this->clearVects();
	x_.push_back(x_0);
	y_.push_back(y_0);
	u_.push_back(u_0);
	v_.push_back(v_0);
	//u.push_back(cos(psi_0)*u_0-sin(psi_0)*v_0);
	//v.push_back(sin(psi_0)*u_0+sin(psi_0)*v_0);
	psi_ = psi_0;
	radius_ = radius;
	
	r11_ = cos(psi_);
	r12_ = -sin(psi_);
	r21_ = sin(psi_);
	r22_ = cos(psi_);

	calculateTrajectory();
};

obstacle::~obstacle()
{
};

void obstacle::calculateTrajectory()
{
	for (int i = 0; i < n_samp_-1;  i++)
	{
		x_.push_back(x_[i] + (r11_*u_[i] + r12_*v_[i])*dt_);
		y_.push_back(y_[i] + (r21_*u_[i] + r22_*v_[i])*dt_);
		u_.push_back(u_[i]);
		v_.push_back(v_[i]);
	}

};

void obstacle::clearVects(){
	x_.clear();
	y_.clear();
	u_.clear();
	v_.clear();
}


	

	
