#include "uav_utils/uav_utils.hpp"

///////////////////
// Define PID class
///////////////////

	//Constructor
	PID::PID (double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(),
		double Tsi = 1.0/100, double Ni = 10)
	{
		Iterm = 0;
		Iprev = 0;
		Eprev = 0;
		Uprev = 0;
		P = Pi;
		I = Ii;
		D = Di;
		satU = satUi;
		satL = satLi;
		Ts = Tsi;
		N = Ni;
	}
	
	double PID::step(double error, double dt)
	{
		double Pterm = P*error;
		Iterm += I*error*dt;
		double Dterm = D / (D + N*dt) * (Uprev + N * (error - Eprev));
		output = Pterm + Iterm + Dterm;
		if (output>satU) {
			output = satU;
			Iterm = Iprev;
		}
		if (output<satL) {
			output = satL;
			Iterm = Iprev;
		}
		Iprev = Iterm;
		Eprev = error;
		Uprev = output;
		return output;
	}
	
	double PID::step(double error, double dt, double derivative)
	{
		double Pterm = P*error;
		Iterm += I*error*dt;
		double Dterm = D / (D + N * dt) * (Uprev + N * derivative * dt);
		output = Pterm + Iterm + Dterm;
		if (output>satU) {
			output = satU;
			Iterm = Iprev;
		}
		if (output<satL) {
			output = satL;
			Iterm = Iprev;
		}
		Iprev = Iterm;
		Uprev = output;
		return output;
	}

	//Destructor
	PID::~PID ()
	{
	}

/////////////////////////////////////////
//Aerodynamc angles/ airspeed calculation
geometry_msgs::Vector3 getAirData (geometry_msgs::Vector3 speeds)
{
	double u = speeds.x;
	double v = speeds.y;
	double w = speeds.z;

	double airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
	double alpha = atan2(w,u);
	double beta = 0;
	if (u==0) {
		if (v==0) {
			beta=0;
		}
		else {
			beta=asin(v/abs(v));
		}

	}
	else {
		beta = atan2(v,u);
	}

	geometry_msgs::Vector3 result;
	result.x = airspeed;
	result.y = alpha;
	result.z = beta;
	
	return result;
}

void WGS84_NM(double lat,double *NE, double *ME)
{
	double const R_earth = 6378137.0;
	double const f_earth = 1.0/298.257223563;
	double const e_earth = sqrt(2.0*f_earth - f_earth*f_earth);
	double sfi =sin(lat);
	double e2 = e_earth*e_earth;

	double temp = 1.0-e2*sfi*sfi;

	*NE = R_earth / sqrt(temp);
	*ME = R_earth*(1.0-e2) / pow(temp,3.0/2.0);
}
