#ifndef OBSTACLE
#define OBSTACLE

#include  <vector>
#include <Eigen/Dense>

class obstacle
{
	public:
	
	/// Constructor
	obstacle(double x_0, double y_0, double u_0, double v_0,  double psi_0, double T, double dt);
	
	/// Destructor
	~obstacle();

	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> u;
	std::vector<double> v;
	double psi;
	
	private:
	
	void calculateTrajectory();
	void clearVects();
	
	int n_samp;
	double T;
	double dt;
	double size;

	//Rotation
	double r11, r12, r21, r22;

};

#endif
