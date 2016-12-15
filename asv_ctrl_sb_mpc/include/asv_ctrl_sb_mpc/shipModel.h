#ifndef SHIPMODEL
#define SHIPMODEL

#include <vector>
#include <Eigen/Dense>

class shipModel
{
	public:
	
	/// Constructor
	shipModel(double T, double dt);
	
	/// Destructor
	~shipModel();
	
	void eulersMethod(Eigen::Vector3d asv_pose, Eigen::Vector3d asv_twist, double u_d, double psi_d);

	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> psi;
	std::vector<double> u;
	std::vector<double> v;
	std::vector<double> r;

	double radius; 	// [m]

	private:
	
	void clearVects();

	void updateCtrlInput(double u_d, double psi_d);

	Eigen::Vector3d eta;
	Eigen::Vector3d nu;
	Eigen::Vector3d tau;

	Eigen::Matrix3d Minv;
	Eigen::Vector3d Cvv;
	Eigen::Vector3d Dvv;

	// Model Parameters
	double M; 	// [kg]
	double I_z; // [kg/m2]

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
	double DT_;
	double T_;
	int n_samp;
	double Kp_u;
	double Kp_psi;
	double Kd_psi;
	double Kp_r;
	
};

#endif
