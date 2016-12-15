#ifndef OBSTACLE
#define OBSTACLE

#include  <vector>
#include <Eigen/Dense>

class obstacle
{
	public:
	
	/// Constructor
	obstacle(double x_0, double y_0, double u_0, double v_0,  double psi_0, double radius, double T, double dt);
	
	/// Destructor
	~obstacle();

	std::vector<double> x_;
	std::vector<double> y_;
	std::vector<double> u_;
	std::vector<double> v_;
	double psi_;
	double radius_;
	
	private:
	
	void calculateTrajectory();
	void clearVects();
	
	int n_samp_;
	double T_;
	double dt_;

	//Elements of the rotation matrix
	double r11_, r12_, r21_, r22_;

};

#endif
