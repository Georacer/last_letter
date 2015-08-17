////////////////////////////////////////////////////
// Electric engine model found in R. Beard's book //
////////////////////////////////////////////////////
class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho;

	///////////
	//Functions
	EngBeard(ModelPlane *, int);
	~EngBeard();

	void updateRadPS(); //Step the angular speed
	void getForce();
	void getTorque();
};
