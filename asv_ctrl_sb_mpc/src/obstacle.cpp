
#include "asv_ctrl_sb_mpc/obstacle.h"

	

obstacle::obstacle(double x, double y, double u,double v, double T, double dt)
{
	T = T;
	dt = dt;
	x_vect.push_back(x); 
	y_vect.push_back(y);
	u_vect.push_back(u);
	v_vect.push_back(v);
	calculateTrajectory();
};

obstacle::~obstacle()
{
};

void obstacle::calculateTrajectory()
{
	int n = T/dt;
	for (int i = 1; i < n;  i++)
	{
		x_vect.push_back(x_vect[i-1] + u_vect[i-1]*dt);
		y_vect.push_back(y_vect[i-1] + v_vect[i-1]*dt);
		u_vect.push_back(u_vect[i-1]);
		v_vect.push_back(v_vect[i-1]);
	}
};
	
std::vector<double> obstacle::getPredX()
{
	return x_vect;
};

std::vector<double> obstacle::getPredY()
{
	return y_vect;
};

std::vector<double> obstacle::getPredU()
{
	return u_vect;
};

std::vector<double> obstacle::getPredV()
{
	return v_vect;
};


double obstacle::getSize()
{
	return size;
};
	
	

	
