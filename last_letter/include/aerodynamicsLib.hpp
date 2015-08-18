/////////////////////////////////////////////
// Aerodynamics class related declarations //
/////////////////////////////////////////////

#include <tf/transform_broadcaster.h>

//////////////////////////////////////////////
// Aerodynamics interface class declaration //
//////////////////////////////////////////////
class Aerodynamics
{
	public:
	////////////
	// Variables
	ModelPlane * parentObj; // Pointer to ModelPlane parent class
	int id; // The current airfoil ID
	geometry_msgs::Vector3 CGOffset; // vector from CG to engine coordinates
	geometry_msgs::Vector3 mountOrientation; // YPR mounting orientation of the wing
	geometry_msgs::Vector3 relativeWind; // relative wind vector in the wing frame
	double airspeed, alpha, beta; // airdata quantities
	double deltaa_max, deltae_max, deltar_max, gimbalAngle_max; // Control inputs and maximum surface deflections
	double inputAileron, inputElevator, inputRudder, inputGimbal;
	int chanAileron, chanElevator, chanRudder, chanGimbal;
	geometry_msgs::Wrench wrenchAero;

	tf::TransformBroadcaster broadcaster; // Transformations broadcaster object
	tf::Transform body_to_mount, mount_to_gimbal, body_to_gimbal; // Transformations in the airfoil assembly for visual rendering
	tf::Transform body_to_mount_rot, mount_to_gimbal_rot, body_to_gimbal_rot; // Transformations in the airfoil assembly for force and moment rotation

	////////////
	// Functions
	Aerodynamics(ModelPlane *, int);
	~Aerodynamics();
	void getInput();
	void stepDynamics(); // perform one step in the aerodynamics
	void rotateWind(); // convert the wind to the propeller axes
	void rotateForce(); // convert the resulting force to the body axes
	void rotateTorque(); // convert the resulting torque to the body axes
	virtual void getForce() = 0;
	virtual void getTorque() = 0;
};


///////////////////////////////////////////
// No aerodynamics, create a dummy class //
///////////////////////////////////////////
class NoAerodynamics : public Aerodynamics{
public:
	NoAerodynamics(ModelPlane *, int);
	~NoAerodynamics();
	void getForce();
	void getTorque();
};

////////////////////////////////////////////////////////
// Typical aerodynamics, linear to their coefficients //
////////////////////////////////////////////////////////
class StdLinearAero : public Aerodynamics
{
	public:
	StdLinearAero(ModelPlane *, int);
	~StdLinearAero();
	double rho,g;
	double c_lift_q,c_lift_deltae,c_drag_q,c_drag_deltae;
	double c,b,s;
	double c_y_0,c_y_b,c_y_p,c_y_r,c_y_deltaa,c_y_deltar;
	double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
	double c_m_0, c_m_a, c_m_q, c_m_deltae;
	double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
	double M, alpha0, c_lift_0, c_lift_a0;
	double c_drag_p, oswald, AR;
	void getForce();
	void getTorque();
	//Calculate lift coefficient from alpha
	virtual double liftCoeff(double);
	//Calculate drag coefficient from alpha
	virtual double dragCoeff(double);
};

////////////////////////////////////////////////////////
// Extension for fine specification of the drag polar //
////////////////////////////////////////////////////////
class HCUAVAero : public StdLinearAero
{
public:
	HCUAVAero(ModelPlane *, int);
	~HCUAVAero();
	Polynomial * liftCoeffPoly;
	Polynomial * dragCoeffPoly;

	double liftCoeff(double alpha);
	double dragCoeff(double alpha);
};