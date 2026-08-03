#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include <Eigen/Eigen>

using Eigen::Vector3d;

namespace last_letter_lib {

//////////////////////////
// Define Gravity class
//////////////////////////

Wrench_t GravityModel::getWrench(const SimState_t states, const Inertial inertial)
{
    calcGravity(states);

    UnitQuaternion orientation_eb = states.pose.orientation.conjugate();
    if (math_utils::isnan(orientation_eb)) { throw runtime_error("gravity.cpp: NaN member in orientation quaternion"); }

    auto gravVect = Vector3d(0, 0, inertial.mass * _g);
    Vector3d force = orientation_eb * gravVect;
    if (math_utils::isnan(force)) { throw runtime_error("gravity.cpp: NaN member in force vector"); }

    Wrench_t wrench(force, Vector3d::Zero());
    return wrench;
}

// Class constructor
GravitySimple::GravitySimple()
{
    setG(9.81);
}

///////////////////
// Calculate gravity
void GravityClassic::calcGravity(const SimState_t states)
{
    double slat2 = pow(sin(M_PI / 180 * states.geoid.latitude), 2);
    double Re2 = pow(Geoid::WGS84_Ra, 2);

    double grav0 = uav_utils::Geoid::EARTH_grav * (1.0 + 0.00193185138639 * slat2) / sqrt(1.0 - 0.00669437999013 * slat2);
    double gravity = grav0 * (1.0 - grav_temp * states.geoid.altitude + 3.0 * (pow(states.geoid.altitude, 2) / Re2));
    if (isnan(gravity)) { throw runtime_error("gravity.cpp: gravity is NaN"); }
    setG(gravity);
}

GravityModel *buildGravity(ParameterManager config)
{
    int gravityType;
    gravityType = config.get<double>("type");
    GravityModel *gravity;
    switch (gravityType)
    {
    case 0:
        gravity = new GravitySimple();
        break;
    case 1:
        gravity = new GravityClassic();
        break;
    default:
        throw runtime_error("Error while constructing gravity");
        break;
    }
    return gravity;
}

} // namespace last_letter_lib
