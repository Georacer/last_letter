#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include "geometry_msgs/Vector3.h"

#include "last_letter/Environment.h"
#include "last_letter/SimStates.h"
#include "mathutils/utls.hpp"

using namespace std;

/////////
//Classes
/////////

class environmentModel
{
	public:
	last_letter::Environment environment;
	last_letter::SimStates states;
	double dt, simRate;
	ros::Time tprev;
	double allowTurbulence;
	
	double g; //Acceleration of gravity at 45.542 deg latitude, m/s**s
	double Rd; //Gas constant for dry air, J/kg K
	//conditions starting at sea level, in a region with temperature gradient
	double T0; //Temperature at sea level, degrees K
	double L0;  //Temperature lapse rate, at sea level deg K/km
	double P0; //Pressure at sea level, in HG
	double Rho0; //Density at sea level, kg/m**3
	//Earth Constants
	double R_earth;
	double f_earth;
	double e_earth;
	double RP_earth;
	double omega_earth;
	double grav_const;
	double grav_earth; //Redefinition for use in calcGrav()
	double Re; //Earth radius
	double grav_temp;
	double windRef, windRefAlt, windDir, surfSmooth, kwind;
	geometry_msgs::Vector3 wind;
	double Lu, Lw, sigmau, sigmaw;
	double windDistU;
	double windDistV[2], windDistW[2];
	
	/////////////
	//Constructor
	environmentModel();
	
	void callback(const last_letter::SimStates::ConstPtr& InpStates);
	
	void calcWind();
	
	void calcDens();
	
	void calcPres();
	
	void calcTemp();
	
	void calcGrav();
};
