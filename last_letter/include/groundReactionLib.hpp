// Ground reactions interface class : Blank
class GroundReaction
{
	public:
	ModelPlane * parentObj;
	GroundReaction(ModelPlane *);
	~GroundReaction();
	geometry_msgs::Wrench wrenchGround;
	virtual geometry_msgs::Vector3 getForce()=0;
	virtual geometry_msgs::Vector3 getTorque()=0;
};

// Panos ground implementation : First pass
class PanosContactPoints : public GroundReaction
{
	public:
	PanosContactPoints(ModelPlane *);
	~PanosContactPoints();
	double* contactPoints, * spp, * sppprev;
	double * cpi_up, * cpi_down, * spd, * pointCoords;
	double uavpos[3], normVe;
	double kspring, mspring, len, frictForw[3], frictSide[3];
	bool contact, safe;
	int contactPtsNo;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};