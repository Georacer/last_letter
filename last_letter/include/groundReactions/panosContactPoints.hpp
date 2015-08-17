/// Panos Marantos ground reactions implementation
class PanosContactPoints : public GroundReaction
{
	public:
	PanosContactPoints(ModelPlane *);
	~PanosContactPoints();
	double * spp, * sppprev;
	double * cpi_up, * cpi_down, * spd, * pointCoords, * materialIndex, * springIndex;
	double uavpos[3], normVe;
	double len, frictForw[4], frictSide[4];
	bool contact, safe;
	int contactPtsNo;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};