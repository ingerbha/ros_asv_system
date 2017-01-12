#include <ros/console.h>

#include "asv_ctrl_sb_mpc/shipModel.h"

//Debug
#include <typeinfo>
#include <cmath>

const float PI = 3.1415927;
static const double DEG2RAD = PI/180.0f;
static const double RAD2DEG = 180.0f/PI;

/// Assures that the numerical difference is at most PI
double normalize_angle_diff(double angle, double angle_ref);

/// Assures that angle is between [-PI, PI)
double normalize_angle(double angle);

shipModel::shipModel(double T, double dt)
{	
	T_ = T;
	DT_ = dt;
	n_samp = floor(T_/DT_);

	eta = Eigen::Vector3d::Zero();
	nu  = Eigen::Vector3d::Zero();
	tau = Eigen::Vector3d::Zero();

	A_ = 5;
	B_ = 5;
	C_ = 2;
	D_ = 2;
	L_ = A_ + B_;
	W_ = C_ + D_;

	calculate_position_offsets();

	radius = 10.0; // [m] (In reality 4.15 m)
	M = 3980.0; // [kg]
	I_z = 19703.0; // [kg/m2]

	// Added M terms
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
	N_r = -1281;//-3224.0;

	// Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
	X_uu = -135.0;
	Y_vv = -2000.0;
	N_rr = 0.0;
	X_uuu = 0.0;
	Y_vvv = 0.0;
	N_rrr = -3224.0;

	Eigen::Matrix3d Mtot;
	Mtot << M - X_udot, 0, 0,
	        0, M-Y_vdot, -Y_rdot,
	        0, -Y_rdot, I_z-N_rdot;
	Minv = Mtot.inverse();

	//Force limits
	Fx_min = -6550.0;
	Fx_max = 13100.0;
	Fy_min = -645.0;
	Fy_max = 645.0;

	// Other
	rudder_d = 4.0; // distance from rudder to CG
	Kp_u = 1.0;
	Kp_psi = 5.0;
	Kd_psi = 1.0;
	Kp_r = 8.0;

}

shipModel::~shipModel(){
}

double shipModel::getL(){
	return L_;
}

double shipModel::getW(){
	return W_;
}


void shipModel::eulersMethod(Eigen::Vector3d asv_pose, Eigen::Vector3d asv_twist, double u_d, double psi_d)
{

	this->clearVects();

	psi.push_back(normalize_angle(asv_pose[2]));
	x.push_back(asv_pose[0] + os_x*cos(psi[0]) - os_y*sin(psi[0]));
	y.push_back(asv_pose[1] + os_x*sin(psi[0]) + os_y*cos(psi[0]));
	u.push_back(asv_twist[0]);
	v.push_back(asv_twist[1]);
	r.push_back(asv_twist[2]);
	
	double t = 0;
	Eigen::Matrix3d rot_z;

	for (int i = 0; i < n_samp-1; i++){

		eta[0] = x[i];
		eta[1] = y[i];
		eta[2] =psi[i];
		nu[0] = u[i];
		nu[1] = v[i];
		nu[2] = r[i];

		t += DT_;
		
		psi_d = normalize_angle_diff(psi_d, psi[i]);

		//rot_z = Eigen::AngleAxisd(eta[2], Eigen::Vector3d::UnitZ());

		rot_z << cos(eta[2]), -sin(eta[2]), 0,
				 sin(eta[2]), sin(eta[2]), 0,
				 0,0,1;

		// Calculate coriolis and dampening matrices according to Fossen, 2011 or Stenersen, 2014.
		Cvv[0] = (-M*nu[1] + Y_vdot*nu[1] + Y_rdot*nu[2]) * nu[2];
		Cvv[1] = ( M*nu[0] - X_udot*nu[0]) * nu[2];
		Cvv[2] = (( M*nu[1] - Y_vdot*nu[1] - Y_rdot*nu[2] ) * nu[0] +
		            ( -M*nu[0] + X_udot*nu[0] ) * nu[1]);

		Dvv[0] = - (X_u + X_uu*fabs(nu[0]) + X_uuu*nu[0]*nu[0]) * nu[0];
		Dvv[1] = - ((Y_v*nu[1] + Y_r*nu[2]) +
		              (Y_vv*fabs(nu[1])*nu[1] + Y_vvv*nu[1]*nu[1]*nu[1]));
		Dvv[2] = - ((N_v*nu[1] + N_r*nu[2]) +
		              (N_rr*fabs(nu[2])*nu[2] + N_rrr*nu[2]*nu[2]*nu[2]));

		this->updateCtrlInput(u_d, psi_d);

		// Integrate system
		eta += DT_ * (rot_z * nu);
		nu  += DT_ * (Minv * (tau- Cvv - Dvv));

		// Keep yaw within [-PI,PI)
		eta[2] = normalize_angle(eta[2]);

		x.push_back(eta[0]);
		y.push_back(eta[1]);
		psi.push_back(eta[2]);
		u.push_back(nu[0]);
		v.push_back(nu[1]);
		r.push_back(nu[2]);

	} 

}

void shipModel::updateCtrlInput(double u_d, double psi_d){
	double Fx = Cvv[0] + Dvv[0] + Kp_u*M*(u_d - nu[0]);
	double Fy = 0.0;

    Fy = (Kp_psi * I_z ) * ((psi_d - eta[2]) - Kd_psi*nu[2]);
    Fy *= 1.0 / rudder_d;

	// Saturate
	if (Fx < Fx_min)
	  Fx = Fx_min;
	if (Fx > Fx_max)
	  Fx = Fx_max;

	if (Fy < Fy_min)
	  Fy = Fy_min;
	if (Fy > Fy_max)
	  Fy = Fy_max;

	tau[0] = Fx;
	tau[1] = Fy;
	tau[2] = rudder_d * Fy;
}

void shipModel::calculate_position_offsets(){
	os_x = A_ - B_;
	os_y = D_ - C_;
}

void shipModel::clearVects(){
	x.clear();
	y.clear();
	psi.clear();
	u.clear();
	v.clear();
	r.clear();
}


double normalize_angle(double angle){

	if( isinf(angle)) return angle;

	while(angle <= -PI) angle += 2*PI;

	while (angle > PI) angle -= 2*PI;

	return angle;
}

double normalize_angle_diff(double angle, double angle_ref){
	double new_angle;
	double diff = angle_ref - angle;

	if (isinf(angle) || isinf(angle_ref)) return angle;

	// Get angle within 2*PI of angle_ref
	if (diff > 0){
		new_angle = angle +(diff - fmod(diff, 2*PI));
	}else{
		new_angle = angle + (diff + fmod(-diff, 2*PI));
	}

	// Get angle on side closest to angle_ref
	diff = angle_ref - new_angle;
	if (diff > PI){
		new_angle += 2*PI;
	}else if (diff < -PI){
		new_angle -= 2*PI;
	}
	return new_angle;
}
