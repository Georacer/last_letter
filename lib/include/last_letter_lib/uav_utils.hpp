#ifndef UAV_UTILS_
#define UAV_UTILS_

#include <cstdio>
#include <cmath>
#include <unordered_map>
#include <Eigen/Eigen>

#include <last_letter_lib/math_utils.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using last_letter_lib::math_utils::UnitQuaternion;
using last_letter_lib::math_utils::Vector3;

namespace last_letter_lib {
namespace uav_utils {

//////////////////
// Data Structures
//////////////////

// Quaternion from ENU inertial frame to NED inertial frame
const UnitQuaternion q_enu_ned = UnitQuaternion(Eigen::AngleAxis<double>(M_PI / 2, Vector3d::UnitZ()) *
                                                Eigen::AngleAxis<double>(M_PI, Vector3d::UnitX()));

const UnitQuaternion q_ned_enu = q_enu_ned.conjugate();
// Quaternion from Gazebo Body frame to Aerospace Body frame
const UnitQuaternion q_bg_ba{Quaterniond{Eigen::AngleAxis<double>(M_PI, Vector3d::UnitX())}};
const UnitQuaternion q_ba_bg = q_bg_ba.conjugate();

struct Twist
{
    Twist() : linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
    Vector3d linear;
    Vector3d angular;
};

class Wrench_t
{
public:
    Wrench_t() : force(Vector3d::Zero()), torque(Vector3d::Zero()) {}
    Wrench_t(const Vector3d force_p, const Vector3d torque_p) : force(force_p), torque(torque_p) {}

    Wrench_t operator+(const Wrench_t &w) const
    {
        auto res = Wrench_t();
        res.force = force + w.force;
        res.torque = torque + w.torque;
        return res;
    }
    Wrench_t operator-(const Wrench_t &w) const
    {
        auto res = Wrench_t();
        res.force = force - w.force;
        res.torque = torque - w.torque;
        return res;
    }

    Eigen::VectorXd to_array() const
    {
        Eigen::VectorXd v(6);
        v << force(0), force(1), force(2),
            torque(0), torque(1), torque(2);
        return v;
    }

    Vector3d force;
    Vector3d torque;
};

struct Pose
{
    Pose() : position(Vector3d::Zero()), orientation(UnitQuaternion()) {}
    Vector3d position;
    UnitQuaternion orientation;
    Pose T() const
    {
        Pose p = Pose();
        p.position = this->orientation * this->position * -1;
        p.orientation = this->orientation.conjugate();
        return p;
    }

    Wrench_t operator*(const Wrench_t w) const
    {
        auto rot_force = orientation * w.force;
        auto lever_arm = position.cross(rot_force);
        auto rot_torque = orientation * w.torque;

        return Wrench_t(rot_force, rot_torque + lever_arm);
    }

    // Used in Pybind11
    void set_position_from_vector3(const Vector3 v)
    {
        position = v.vector;
    }
    Vector3 get_position_as_vector3()
    {
        return Vector3(position.x(), position.y(), position.z());
    }
    void set_orientation_from_vector(const Vector4d v)
    {
        orientation.w() = v(0);
        orientation.x() = v(1);
        orientation.y() = v(2);
        orientation.z() = v(3);
    }
    Vector4d get_orientation_as_vector()
    {
        return Vector4d(
            orientation.w(),
            orientation.x(),
            orientation.y(),
            orientation.z());
    }
};

typedef std::unordered_map<std::string, Wrench_t> LinkWrenchMap_t;

class WrenchSum_t
{
public:
    WrenchSum_t(): wrenchGrav(), wrenchAero(), wrenchProp(), wrenchGround(), wrenchExternal() {};
    Wrench_t sum() const; // Sum the wrench components.
    WrenchSum_t operator+(const WrenchSum_t w) const;
    WrenchSum_t &operator+=(const WrenchSum_t w);

    Wrench_t wrenchGrav;
    Wrench_t wrenchAero;
    Wrench_t wrenchProp;
    Wrench_t wrenchGround;
    Wrench_t wrenchExternal; // For any unmodelled or independent external disturbances
};

struct Accelerations
{
    Accelerations() : linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
    Vector3d linear;
    Vector3d angular;
};

struct Geoid
{
    Geoid() : latitude(0), longitude(0), altitude(0), velocity(Vector3d::Zero()) {}
    constexpr static double WGS84_Ra = 6378137.0; // Earth ellipsoid semi-major axis (alpha);
    constexpr static double EARTH_flattening = 0.003352811;
    constexpr static double WGS84_e2 = 0.006694380022901;
    constexpr static double EARTH_Omega = 7.292115e-5;
    constexpr static double EARTH_grav = 9.7803267714;
    double latitude;
    double longitude;
    double altitude;
    Vector3d velocity; // in m/s North, East, Up
};

class SimState_t
{
public:
    SimState_t() : pose(), velocity(), rotorspeed(4, 0.01) {}
    SimState_t(Vector3d position,
                UnitQuaternion orientation,
                Vector3d velocity_linear,
                Vector3d velocity_angular,
                std::vector<double> thrusters_velocity = std::vector<double>(0));
    SimState_t(Vector3 position,
                UnitQuaternion orientation,
                Vector3 velocity_linear,
                Vector3 velocity_angular,
                std::vector<double> thrusters_velocity = std::vector<double>(0));
    // Decode a SimState_t from a vector as
    // 0-2: position
    // 3-6: orientation quaternion
    // 7-9: linear velocity
    // 10-12: angular velocity
    // 12-: thruster velocity
    // This constructor is problematic because it doesn't contain all of the class elements,
    // but is implemented for pickling the Python bound object.
    SimState_t(const VectorXd);
    SimState_t(const SimState_t &) = default;
    // Inverse of the constructor from VectorXd.
    VectorXd to_array();
    SimState_t strip_thrusters();
    Vector3d get_position() { return pose.position; }
    void set_position(const Vector3d &v) { pose.position = v; }
    UnitQuaternion get_orientation() { return pose.orientation; }
    void set_orientation(const UnitQuaternion &q) { pose.orientation = q; }
    Vector3d get_velocity_linear() { return velocity.linear; }
    void set_velocity_linear(const Vector3d &v) { velocity.linear = v; }
    Vector3d get_velocity_angular() { return velocity.angular; }
    void set_velocity_angular(const Vector3d &v) { velocity.angular = v; }
    Geoid geoid;
    Pose pose;						// NED. The quaternion offers the Body-to-Earth transformation.
    Twist velocity;					// NED
    Accelerations acceleration;		// NED
    std::vector<double> rotorspeed; // in rad/s
};

typedef std::unordered_map<std::string, SimState_t> LinkStateMap_t;

// Number of control channels carried by an Input / InputPwm_t.
constexpr size_t num_input_channels{32};

class Input
{
public:
    Input() : value(num_input_channels, 0) {}
    // Build from a channel vector. Shorter inputs are zero-padded to num_input_channels.
    Input(const std::vector<double> &v);
    std::vector<double> value; // -1 - 1 (or 0 - 1) ranged
    std::vector<double> to_array() const { return value; }
    static Input from_array(const std::vector<double> &v) { return Input(v); }
};

struct InputPwm_t
{
    InputPwm_t() : value(num_input_channels, 1000) {}
    std::vector<uint16_t> value; // PWM microseconds, 1000-2000 typical range
};

// Air data class declaration
class Airdata
{
public:
    Airdata(double airspeed_p = 0, double alpha_p = 0, double beta_p = 0)
        : airspeed(airspeed_p), alpha(alpha_p), beta(beta_p){};
    ~Airdata(){};
    /*
    Calculate the relative air data from inertial and wind speeds

    INPUTS:
        v_b: Inertial velocity, body-frame
        v_w: Wind (air-mass) velocity, body-frame

    OUTPUTS:
        airspeed: The norm of the relative wind
        alpha: The angle of attack
        beta: The angle of sideslip
    */
    void init_from_velocity(Vector3d velBody, Vector3d velWind = Vector3d::Zero());
    // Calculate airdata given the UAV state and the environment data.
    void init_from_state_wind(SimState_t, Vector3d);
    /*
    Create rotation matrix S, which transforms from body frame to wind frame
    Taken from Aircraft Control and Simulation, Stevens Lewis, p.63
    */
    Eigen::Matrix3d S_bw();
    // Create rotation matrix S^T, which transforms from wind frame to body frame
    Eigen::Matrix3d S_wb();
    // Convert airdata into body-frame velocities
    Vector3d to_velocity();
    Vector3d to_vector3d() { return Vector3d(airspeed, alpha, beta); }
    std::string str();

    double airspeed{0}; // relative airspeed
    double alpha{0};	// angle of attach
    double beta{0};		// angle of sideslip
};

////////////////////////////
// Kinematic Transformations
////////////////////////////

Vector3d getEulerDerivatives(Vector3d euler, Vector3d rates);
Vector3d getAngularRatesFromEulerDerivatives(Vector3d euler, Vector3d eulerDot);

//////////////
// Controllers
//////////////

class PID
{
private:
public:
    ///////////
    // Variables
    double P, I, D, satU, satL, Ts, Tau, trim;
    double Iterm, Iprev, Eprev, Uprev, Dprev;
    double output;
    ///////////
    // Functions

    // Constructor
    PID(double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

    // Destructor
    ~PID();

    // Main step
    double step(double error);
    double step(double error, double dt);
    double step(double error, double dt, double derivative);
    void init(void);
};

class APID
{
public:
    ///////////
    // Variables
    double P, I, D, satU, satL, Ts, Tau, trim;
    double Pinit, Iinit, Dinit;
    double Iterm, Iprev, Eprev, Uprev, Dprev;
    double Ierror, bumplessI1, bumplessI2, trErr;
    double output;
    ///////////
    // Functions

    // Constructor
    APID(double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

    // Destructor
    ~APID();

    // Main step
    double step(double error, bool track, double trInput);
    void init(void);
};

/////////////////////////////
// WGS84 utility functions //
/////////////////////////////

/**
    * [WGS84_RN Calculate the curvature of the Earth in the prime vertical
    * @param  lat The latitude in degrees
    * @return     RN - radius corresponding to curvature in meters
    */
double WGS84_RN(double lat);

/**
    * [WGS84_RM Calculate the curvature of the Earth in the meridian
    * @param  lat The latitude in degrees
    * @return     RM - radius corresponding to curvature in meters
    */
double WGS84_RM(double lat);

/////////////////////////////
// Other navigation functions
/////////////////////////////

/**
    * @brief Get latitude and longitude coordinates fromlocal position.
    * @details Taken from https://github.com/PX4/PX4-SITL_gazebo/blob/97106007eb5c934b902a5329afb55b45e94d5063/include/common.h
    * Makes the small-angle assumption and disregards altitude.
    *
    * @param pos_north Local position North
    * @param pos_east Local position East
    * @param lat_home Home (initialization) latitude, in degrees
    * @param lon_home Home (initialization) longitude, in degrees
    * @return std::pair<double, double> The new (lat, lon) coordinates, in degrees
    */
std::pair<double, double> reproject(const double pos_north,
                                    const double pos_east,
                                    const double lat_home,
                                    const double lon_home);

//////////////////////////////
// PPM and PWM functionalities
//////////////////////////////

// Plausible bounds for a servo command. 1000-2000 us is the nominal range; the
// margin allows for extended endpoints. Values outside it are not commands at
// all: an autopilot typically emits 0 us on a channel it is not driving, and
// PwmToFullRange(0) is -3, i.e. triple full deflection.
constexpr uint16_t min_valid_pwm{800};
constexpr uint16_t max_valid_pwm{2200};

inline bool IsValidPwm(uint16_t pwmValue)
{
    return pwmValue >= min_valid_pwm && pwmValue <= max_valid_pwm;
}

double PwmToHalfRange(uint16_t pwmValue);
// Convert a 1000-2000 us value to 0-1 range

double PwmToFullRange(uint16_t pwmValue);
// Convert a 1000-2000 us value to -1-1 range

uint16_t HalfRangeToPwm(double signal);
// Convert a 0-1 range to 1000-2000 us range

uint16_t FullRangeToPwm(double signal);
// Convert a -1-1 range to 1000-2000 us range

//////////////////////////
// Miscellaneous Utilities
//////////////////////////

} // namespace uav_utils
} // namespace last_letter_lib

#endif
