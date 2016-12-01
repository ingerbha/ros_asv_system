#include <ros/console.h>

#include <cmath>

#include "asv_ctrl_sb_mpc/obstacle.h"

	

obstacle::obstacle(double x_0, double y_0, double u_0, double v_0,  double psi_0, double T_, double dt_)
{
	T = T_;
	dt = dt_;
	n_samp = T/dt;
	
	this->clearVects();
	x.push_back(x_0);
	y.push_back(y_0);
	u.push_back(u_0);
	v.push_back(v_0);
	//u.push_back(cos(psi_0)*u_0-sin(psi_0)*v_0);
	//v.push_back(sin(psi_0)*u_0+sin(psi_0)*v_0);
	psi = psi_0;
	
	r11 = cos(psi);
	r12 = -sin(psi);
	r21 = sin(psi);
	r22 = cos(psi);

	calculateTrajectory();
};

obstacle::~obstacle()
{
};

void obstacle::calculateTrajectory()
{
	for (int i = 0; i < n_samp-1;  i++)
	{
		x.push_back(x[i] + (r11*u[i] + r12*v[i])*dt);
		y.push_back(y[i] + (r21*u[i] + r22*v[i])*dt);
		u.push_back(u[i]);
		v.push_back(v[i]);
	}

};

void obstacle::clearVects(){
	x.clear();
	y.clear();
	u.clear();
	v.clear();
}


	

	
