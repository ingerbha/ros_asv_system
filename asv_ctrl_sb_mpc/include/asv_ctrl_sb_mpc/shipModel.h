#ifndef SHIPMODEL
#define SHIPMODEL

#include <vector>
#include <Eigen/Dense>

class shipModel
{
	public:
	
	/// Constructor
	shipModel();
	
	/// Destructor
	~shipModel();
	
	void eulersMethod(Eigen::Vector3d asv_pose, Eigen::Vector3d asv_twist, double u_d, double psi_d);
	
	std::vector<double> getXVect();
	
	std::vector<double> getYVect();
	
	std::vector<double> getUVect();
	
	std::vector<double> getVVect();
	
	std::vector<double> getPsiVect();
	
	private:
	
	// Model Parameters
	double radius; 	// [m] 
	double mass; 	// [kg]
	double inertia; // [kg/m2]

	// Added mass terms
	double X_udot;
	double Y_vdot;
	double Y_rdot;
	double N_vdot;
	double N_rdot;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	double X_u;
	double Y_v;
	double Y_r;
	double N_v;
	double N_r;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	double X_uu;
	double Y_vv;
	double N_rr;
	double X_uuu;
	double Y_vvv;
	double N_rrr;

	//Force limits
	double Fx_min;
	double Fx_max;
	double Fy_min;
	double Fy_max;

	// Other
	double rudder_d;

	// Simulation parameters
	const double DT;
	const double T;
	double Cx;
	double Cy;
	
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> u;
	std::vector<double> v;
	std::vector<double> r;
	std::vector<double> psi;
	std::vector<double> nc;
	std::vector<double> dc;
	
};

#endif
