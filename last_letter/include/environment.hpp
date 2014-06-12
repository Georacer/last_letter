#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include <geometry_msgs/Vector3.h>

#include <last_letter/Environment.h>
#include <last_letter/SimStates.h>
#include <last_letter/Geoid.h>
#include <mathutils/utls.hpp>

using namespace std;

#define E_EARTH sqrt(2.0*last_letter::Geoid::EARTH_flattening - last_letter::Geoid::EARTH_flattening*last_letter::Geoid::EARTH_flattening)
#define RP_EARTH last_letter::Geoid::EARTH_radius*(1.0-E_EARTH*E_EARTH)

#define grav_const 3.986004418e14		
#define grav_temp (2.0/last_letter::Geoid::EARTH_radius)*(1.0+last_letter::Geoid::EARTH_flattening+(last_letter::Geoid::EARTH_Omega*last_letter::Geoid::EARTH_Omega)*(last_letter::Geoid::EARTH_radius*last_letter::Geoid::EARTH_radius)*RP_EARTH/grav_const)

#define Rd 287.05307  //Gas constant for dry air, J/kg K
#define L0 -6.5  //Temperature lapse rate, at sea level deg K/km

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

	ros::Publisher env_pub;
	
	//conditions starting at sea level, in a region with temperature gradient
	double T0; //Temperature at sea level, degrees K
	double P0; //Pressure at sea level, in HG
	double Rho0; //Density at sea level, kg/m**3
	double grav0; //Surface earth gravity
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
