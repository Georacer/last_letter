//////////////////
// No Engine Model
//////////////////

// Constructor
NoEngine::NoEngine(ModelPlane * parent, int id) : Propulsion(parent, id)
{
	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	omega = 0.0;
}

// Destructor
NoEngine::~NoEngine()
{
}

void NoEngine::updateRadPS()
{
}

// Force calculation function
void NoEngine::getForce()
{
}

// Torque calculation function
void NoEngine::getTorque()
{
}