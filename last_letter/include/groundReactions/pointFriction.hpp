/// Elementary point-friction model
class PointFriction : public GroundReaction
{
	/// Does not include rolling wheel model
	public:
	PointFriction(ModelPlane *);
	~PointFriction();
	double * spp, * sppprev;
	double * cpi_up, * cpi_down, * spd, * pointCoords, * materialIndex, * springIndex;
	double uavpos[3], normVe;
	double len, frict[4];
	bool contact, safe;
	int contactPtsNo;
	geometry_msgs::Vector3 extForce, extTorque;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};