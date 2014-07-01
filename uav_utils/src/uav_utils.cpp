#include "uav_utils/uav_utils.hpp"
#include <cmath>

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
