#include "last_letter_lib/kinematics.hpp"

namespace last_letter_lib {

//////////////////////////
// Define Kinematics class
//////////////////////////

//////////////////
// Class Destructor
Kinematics::~Kinematics()
{
    if (integrator != nullptr) {
        delete integrator;
    }
}

void Kinematics::initialize_parameters()
{
    set_param("world/deltaT", 0.0025, false);
}

void Kinematics::update_parameters()
{
    dt = get_param<double>("world/deltaT");
    if (integrator != nullptr) {
        integrator->dt = dt;
    }
}

void Kinematics::set_dt(double dt_p)
{
    dt = dt_p;
    set_param("world/deltaT", dt_p);
    if (integrator != nullptr) {
        integrator->dt = dt_p;
    }
}

void Kinematics::initialize(ParameterManager config)
{
    Parametrized::initialize(config);

    // Build the integrator object
    if (integrator == nullptr) {
        integrator = buildIntegrator(config.filter("world/"));
    }
}

///////////////////////////////
// State derivatives calculation
void Kinematics::calcDerivatives(SimState_t states, Inertial inertial, Wrench_t inpWrench)
{
    // variable declaration
    // create position derivatives from earth velocity
    stateDot.posDot = states.pose.orientation * states.velocity.linear;
    if (!stateDot.posDot.allFinite())
    {
        throw runtime_error("NaN member in position derivative vector");
    }

    // create body velocity derivatives from acceleration, angular rotation and body velocity
    Vector3d linearAcc = (1.0 / inertial.mass) * inpWrench.force;
    if (!linearAcc.allFinite())
    {
        throw runtime_error("NaN member in linear acceleration vector");
    }
    Vector3d corriolisAcc = -states.velocity.angular.cross(states.velocity.linear);
    if (!corriolisAcc.allFinite())
    {
        throw runtime_error("NaN member in corriolis acceleration vector");
    }
    stateDot.speedDot = linearAcc + corriolisAcc;

    // create angular derivatives quaternion from angular rates
    // ATTENTION! This quaternion derivative equation refers to the body-to-earth quaternion
    double x, y, z, w;
    x = states.velocity.angular.x() * 0.5 * dt;
    y = states.velocity.angular.y() * 0.5 * dt;
    z = states.velocity.angular.z() * 0.5 * dt;
    w = 1.0;
    stateDot.quatDot = Quaterniond(w, x, y, z);
    if (!math_utils::myisfinite(stateDot.quatDot))
    {
        throw runtime_error("NaN member in quaternion derivative vector");
    }

    // create angular rate derivatives from torque
    stateDot.rateDot = inertial.tensor.inverse() * (inpWrench.torque - states.velocity.angular.cross(inertial.tensor * states.velocity.angular));
    if (!stateDot.rateDot.allFinite())
    {
        throw runtime_error("NaN member in angular velocity derivative vector");
    }

    stateDot.coordDot.x() = 180.0 / M_PI * asin(stateDot.posDot.x() / WGS84_RM(states.geoid.latitude));
    stateDot.coordDot.y() = 180.0 / M_PI * asin(stateDot.posDot.y() / WGS84_RN(states.geoid.longitude));
    stateDot.coordDot.z() = -stateDot.posDot.z();
}

//////////////////////////
// Propagate the UAV state
SimState_t Kinematics::propagateState(SimState_t states, Inertial inertia, Wrench_t inpWrench)
{
    calcDerivatives(states, inertia, inpWrench);
    return integrator->propagation(states, stateDot);
}

//////////////////////////
// Define Integrator class
//////////////////////////

Integrator::Integrator(ParameterManager worldConfig)
{
    dt = worldConfig.get<double>("deltaT");
}

Integrator::~Integrator()
{
}

//////////////////////////
// Define ForwardEuler class
//////////////////////////

// Class Constructor
ForwardEuler::ForwardEuler(ParameterManager worldConfig) : Integrator(worldConfig)
{
}

// Propagation of the states
SimState_t ForwardEuler::propagation(SimState_t states, Derivatives_t derivatives)
{
    SimState_t newStates;

    // Propagate the NED coordinates from earth velocity
    newStates.pose.position = states.pose.position + derivatives.posDot * dt;
    if (!newStates.pose.position.allFinite())
    {
        throw runtime_error("NaN member in position vector");
    }

    // Propagate orientation quaternion from angular derivatives quaternion
    Quaterniond tempQuat = states.pose.orientation * derivatives.quatDot;
    // Quaterniond tempQuat = states.pose.orientation.conjugate()*derivatives.quatDot;
    newStates.pose.orientation = tempQuat.normalized();
    if (!math_utils::myisfinite(newStates.pose.orientation))
    {
        throw runtime_error("NaN member in orientation quaternion");
    }

    // Propagate body velocity from body velocity derivatives
    newStates.velocity.linear = states.velocity.linear + derivatives.speedDot * dt;
    if (!newStates.velocity.linear.allFinite())
    {
        throw runtime_error("NaN member in linear velocity vector");
    }

    // Propagate angular velocity from angular derivatives
    newStates.velocity.angular = states.velocity.angular + derivatives.rateDot * dt;
    if (!newStates.velocity.angular.allFinite())
    {
        throw runtime_error("NaN member in angular velocity vector");
    }

    // Set linear acceleration from the speed derivatives
    newStates.acceleration.linear = derivatives.speedDot;

    // Set angular acceleration from the angular rate derivatives
    newStates.acceleration.angular = derivatives.rateDot;

    // Update Geoid stuff using the NED coordinates
    newStates.geoid.latitude = states.geoid.latitude + derivatives.coordDot.x() * dt;
    newStates.geoid.longitude = states.geoid.longitude + derivatives.coordDot.y() * dt;
    newStates.geoid.altitude = states.geoid.altitude + derivatives.coordDot.z() * dt;
    newStates.geoid.velocity[0] = derivatives.posDot.x();
    newStates.geoid.velocity[1] = derivatives.posDot.y();
    newStates.geoid.velocity[2] = -derivatives.posDot.z(); // Upwards velocity

    return newStates;
}

// Build integrator model
Integrator *buildIntegrator(ParameterManager worldConfig)
{
    int integratorType;
    integratorType = worldConfig.get<int>("integratorType");
    std::cout << "building integrator class... ";
    switch (integratorType)
    {
    case 0:
        std::cout << "selecting Forward Euler" << std::endl;
        return new ForwardEuler(worldConfig);
    default:
        throw runtime_error("Invalid integrator type!");
        break;
    }
}

} // namespace last_letter_lib
