///////////////////
// Piston engine //
///////////////////
class PistonEng : public Propulsion
{
public:
	////////////
	// Variables
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;

	//////////
	// Members
	Polynomial * engPowerPoly;
	Polynomial * npPoly;
	Polynomial * propPowerPoly;
	// Polynomial1D * npPoly;
	// Polynomial1D * propPowerPoly;
	// Polynomial2D * engPowerPoly;

	////////////
	// Functions
	PistonEng(ModelPlane *, int);
	~PistonEng();

	void updateRadPS();
	void getForce();
	void getTorque();
};