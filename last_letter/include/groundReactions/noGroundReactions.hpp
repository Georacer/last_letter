/// No ground reactions, dummy class
class NoGroundReaction : public GroundReaction
{
public:
	///This is the NoGroundReaction constructor brief description
	NoGroundReaction(ModelPlane *);
	~NoGroundReaction();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};