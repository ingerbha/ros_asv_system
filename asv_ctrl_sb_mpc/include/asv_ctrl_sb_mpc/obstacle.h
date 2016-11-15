#ifndef OBSTACLE
#define OBSTACLE

#include  <vector>
#include "asv_msgs/State.h"

class obstacle
{
	public:
	
	/// Constructor
	obstacle(double x, double y, double u,double v, double T, double dt);
	
	/// Destructor
	~obstacle();
	
	std::vector<double> getPredX();
	std::vector<double> getPredY();
	std::vector<double> getPredU();
	std::vector<double> getPredV();
	double getSize();
	
	private:
	
	void calculateTrajectory();
	
	asv_msgs::State state;
	
	double T;
	double dt;
	double size;
	std::vector<double> x_vect;
	std::vector<double> y_vect;
	std::vector<double> u_vect;
	std::vector<double> v_vect;

};

#endif
