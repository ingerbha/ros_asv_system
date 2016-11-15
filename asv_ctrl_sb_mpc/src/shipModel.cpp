#include "asv_ctrl_sb_mpc/shipModel.h"

// Remember to include in CMake file

shipModel::shipModel(): DT(0.1),
						T(100) // 
{	
	radius = 10.0; // [m] (In reality 4.15 m)
	mass = 3980.0; // [kg]
	inertia = 19703.0; // [kg/m2]

	// Added mass terms
	X_udot = 0.0;
	Y_vdot = 0.0;
	Y_rdot = 0.0;
	N_vdot = 0.0;
	N_rdot = 0.0;

	// Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
	X_u	= -50.0;
	Y_v = -200.0;
	Y_r = 0.0;
	N_v = 0.0;
	N_r = -3224.0;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	X_uu = -135.0;
	Y_vv = -2000.0;
	N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;

	//Force limits
	Fx_min = -6550.0;
	Fx_max = 13100.0;
	Fy_min = -645.0;
	Fy_max = 645.0;

	// Other
	rudder_d = 4.0; // distance from rudder to CG
	rudder_d = 4.0; // distance from rudder to CG

	// Simulation parameters
	Cx = 39.1;
	Cy = 570;
};

std::vector<double> shipModel::getXVect()
{
	return x;
};

std::vector<double> shipModel::getYVect()
{
	return y;
};

std::vector<double> shipModel::getUVect()
{
	return psi;
};

std::vector<double> shipModel::getVVect()
{
	return v;
};

std::vector<double> shipModel::getPsiVect()
{
	return psi;
};


void shipModel::eulersMethod(Eigen::Vector3d asv_pose, Eigen::Vector3d asv_twist, double u_d, double psi_d)
{
	x.push_back(asv_pose[1]);
	y.push_back(asv_pose[2]);
	psi.push_back(asv_pose[3]);
	u.push_back(asv_twist[1]);
	v.push_back(asv_twist[2]);
	r.push_back(asv_twist[3]);
	nc.push_back(0); // TODO: change init cond 
	dc.push_back(0); // TODO: change init cond 
	

	// TODO: Sort this out
	double t = 0;
	int n = T/DT;
	double mX, mY, IN, Fx, Fy;
	double x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot, nc_dot, dc_dot;
	for (int i = 0; i < n; i++){
		t += DT;
		
		Fx = Cx*nc[i];
		Fy = Cy*abs(dc[i])*dc[i];
		
		if (Fx > Fx_max){
			Fx = Fx_max;
		}else if (Fx < Fx_min){
			Fx = Fx_min;
		}
		if (Fy > Fy_max){
			Fy = Fy_max;
		}else if (Fy < Fy_min){
			Fy = Fy_min;
		}
		
		mX = mass*v[i]*r[i] + X_u*u[i] + X_uu*abs(u[i])*u[i] + Fx;
		mY = -mass*u[i]*r[i] + Y_v*v[i] + Y_r*r[i] + Y_vv*abs(v[i])*v[i] + Fy;
		IN = N_v*v[i] + N_r*r[i] + N_rr*abs(r[i])*r[i] + N_rrr*pow(r[i],3) + rudder_d*Fy;
		
		x_dot = cos(psi[i])*u[i] - sin(psi[i])*v[i];
		y_dot = sin(psi[i])*u[i] + cos(psi[i])*v[i];
		psi_dot = r[i];
		u_dot = mX/mass;
		v_dot = mY/mass;
		r_dot = IN/inertia;
		nc_dot = -mX/mass + u_d - u[i];
		dc_dot = -0.6*r[i] + 0.001*(psi_d - psi[i]); //TODO: save constants elsewhere
		
		x.push_back(x[i]+ DT*x_dot);
		y.push_back(y[i]+ DT*y_dot);
		psi.push_back(psi[i]+ DT*psi_dot);
		u.push_back(u[i]+ DT*u_dot);
		v.push_back(v[i]+ DT*v_dot);
		r.push_back(r[i]+ DT*r_dot);
		nc.push_back(nc[i]+ DT*nc_dot);
		dc.push_back(dc[i]+ DT*dc_dot);
		
	} 
};
