#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>
#include <cmath>

#include "geometry_msgs/Vector3.h"

class PID
{
	private:
	public:
	///////////
	//Variables
	double P, I, D, satU, satL, Ts, N;
	double Iterm, Iprev, Eprev, Uprev;
	double output;
	///////////
	//Functions
	
	//Constructor
	PID (double Pi, double Ii, double Di, double satUi, double satLi, double Tsi, double Ni);

	//Destructor
	~PID ();
	
	//Main step
	double step(double error, double dt);
	double step(double error, double dt, double derivative);
};

void WGS84_NM(double lat,double *NE, double *ME);
