////////////////////////////
// No engine, dummy class //
////////////////////////////
class NoEngine: public Propulsion
{
public:
	NoEngine(ModelPlane *, int);
	~NoEngine();

	void updateRadPS();
	void getForce();
	void getTorque();
};