// Gravity class : First pass
class Gravity
{
	public:
	ModelPlane * parentObj;
	Gravity(ModelPlane *);
	~Gravity();
	double col_x, col_y, col_z;
	double g;
	geometry_msgs::Wrench wrenchGrav;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};