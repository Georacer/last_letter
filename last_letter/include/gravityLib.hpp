// Gravity class declaration

class Gravity
{
	public:
	ModelPlane * parentObj; // pointer to parent ModelPlane class
	Gravity(ModelPlane *);
	~Gravity();
	double col_x, col_y, col_z;
	double g;
	geometry_msgs::Wrench wrenchGrav;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};