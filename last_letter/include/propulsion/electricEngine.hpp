///////////////////////////
// Electric hobby engine //
///////////////////////////

class ElectricEng : public Propulsion
{
public:
	////////////////
	// Variables //
	////////////////
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;
	double Kv, Rm, I0;
	// Battery specification
	int Cells; // Number of LiPo cells
	double Rs; // Battery internal resistance
	last_letter_msgs::ElectricEng message;
	ros::Publisher pub;

	//////////////
	// Members //
	//////////////
	Polynomial * engPowerPoly, * npPoly, * propPowerPoly;

	////////////////
	// Functions //
	////////////////
	ElectricEng(ModelPlane *, int);
	~ElectricEng();

	void updateRadPS();
	void getForce();
	void getTorque();
};