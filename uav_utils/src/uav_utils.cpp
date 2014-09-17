#include "uav_utils/uav_utils.hpp"

///////////////////
// Define PID class
///////////////////

	//Constructor
	PID::PID (double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
		double Tsi = 1.0/100, double Taui = 0.1)
	{
		init();
		P = Pi;
		I = Ii;
		D = Di;
		satU = satUi;
		satL = satLi;
		trim = trimi;
		Ts = Tsi;
		Tau = Taui;
	}
	
	void PID::init(void) {
		Iterm = 0;
		Iprev = 0;
		Eprev = 0;
		Uprev = 0;
		Dprev = 0;
	}
	
	double PID::step(double error)
	{
	
		Iterm += I*Ts*error;
		double Dterm = 1.0 / (Tau + Ts) * ( Tau * Dprev + error - Eprev);
		output = P*error + Iterm + D*Dterm + trim;

//		double Pterm = P*error;
//		Iterm += Ts/2 * (error + Eprev);
//		double Dterm = (2*Tau - Ts)/(2*Tau + Ts) * Dprev + 2/(2*Tau + Ts)*(error - Eprev); //Bilinear transform, not working
//		output = Pterm + I*Iterm + D*Dterm;
		
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
		Dprev = Dterm;
		return output;
	}
	
	double PID::step(double error, double dt)
	{
		double Pterm = P*error;
		Iterm += I*error*dt;
		double Dterm = D / (D + dt/Tau) * (Uprev + (error - Eprev)/Tau);
		output = Pterm + Iterm + Dterm + trim;
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
		double Dterm = D / (D + dt/Tau) * (Uprev + (error - Eprev)/Tau);
		output = Pterm + Iterm + Dterm + trim;
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

///////////////////
// Define APID class
///////////////////

	//Constructor
	APID::APID (double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
		double Tsi = 1.0/100, double Taui = 0.1)
	{
		init();
		Pinit = Pi;
		P = Pinit;
		Iinit = Ii;
		I = Iinit;
		Dinit = Di;
		D = Dinit;
		satU = satUi;
		satL = satLi;
		trim = trimi;
		Ts = Tsi;
		Tau = Taui;
	}
	
	void APID::init(void) {
		Iterm = 0;
		Iprev = 0;
		Eprev = 0;
		Uprev = 0;
		Dprev = 0;
		Ierror = 0;
		bumplessI1 = 0;
		bumplessI2 = 0;
	}
	
	double APID::step(double error, bool track, double trInput)
	{
		double PGainPrev = P;
		double IGainPrev = I;
		double DGainPrev = I;
		if (!track) {
			Ierror += error*Ts;
			// Ierror = std::max(-100.0, std::min(100.0, Ierror));

			P += (0.000001*error*error - 0.01*P)*Ts;
			I += (1e-7*Ierror*Ierror - 0.5*I)*Ts;
			// I += (0.00001*error*Ierror)*Ts;
			D += (1e-8*pow((error - Eprev)/Ts,2) - 0.05*D)*Ts;
			if (I<Iinit) {
				I = Iinit;
			}
			bumplessI1 = 0;
			bumplessI2 = 0;
		}

		Iterm += I*Ts*error;
		output = P*error + Iterm + D*(error-Eprev)/Ts + trim;
		// output = P*error + Iterm + trim;


		if (!track) {
			if (output>satU) {
				output = satU;
				Iterm = Iprev;
				P = PGainPrev;
				I = IGainPrev;
				D = DGainPrev;
			}
			if (output<satL) {
				output = satL;
				Iterm = Iprev;
				P = PGainPrev;
				I = IGainPrev;
				D = DGainPrev;
			}
		}

		if (track) {
			trErr = 100*(trInput - output);
			bumplessI1 += 25*trErr*Ts;
			bumplessI2 += 2.0*bumplessI1*Ts;
			Iterm += (trErr + bumplessI1 + bumplessI2)*Ts;
			Ierror = 0;
			P = Pinit;
			I = Iinit;
			D = Dinit;
		}

		Iprev = Iterm;
		Eprev = error;
		return output;
	}
	
	//Destructor
	APID::~APID ()
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
