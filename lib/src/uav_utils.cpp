#include <algorithm>
#include <stdexcept>

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/math_utils.hpp"

using Eigen::Quaterniond;
using Eigen::Vector3d;

using last_letter_lib::math_utils::rad_to_deg;

namespace last_letter_lib {
namespace uav_utils {

Wrench_t WrenchSum_t::sum() const
{
    auto result = Wrench_t();
    result.force = wrenchGrav.force + wrenchAero.force + wrenchProp.force + wrenchGround.force + wrenchExternal.force;
    result.torque = wrenchGrav.torque + wrenchAero.torque + wrenchProp.torque + wrenchGround.torque + wrenchExternal.torque;
    return result;
}

WrenchSum_t WrenchSum_t::operator+(const WrenchSum_t w) const
{
    auto result = WrenchSum_t();
    result.wrenchGrav = wrenchGrav + w.wrenchGrav;
    result.wrenchAero = wrenchAero + w.wrenchAero;
    result.wrenchProp = wrenchProp + w.wrenchProp;
    result.wrenchExternal = wrenchExternal + w.wrenchExternal;
    return result;
}

WrenchSum_t &WrenchSum_t::operator+=(const WrenchSum_t w)
{
    wrenchGrav = wrenchGrav + w.wrenchGrav;
    wrenchAero = wrenchAero + w.wrenchAero;
    wrenchProp = wrenchProp + w.wrenchProp;
    wrenchExternal = wrenchExternal + w.wrenchExternal;
    return *this;
}

SimState_t::SimState_t(Vector3d position,
                        UnitQuaternion orientation,
                        Vector3d velocity_linear,
                        Vector3d velocity_angular,
                        std::vector<double> thrusters_velocity)
{
    pose.position = position;
    pose.orientation = orientation;
    velocity.linear = velocity_linear;
    velocity.angular = velocity_angular;
    rotorspeed = thrusters_velocity;
}
SimState_t::SimState_t(Vector3 position,
                        UnitQuaternion orientation,
                        Vector3 velocity_linear,
                        Vector3 velocity_angular,
                        std::vector<double> thrusters_velocity)
    : SimState_t(position.vector,
                    orientation,
                    velocity_linear.vector,
                    velocity_angular.vector,
                    thrusters_velocity)
{
}
SimState_t::SimState_t(const VectorXd v)
{
    pose.position = Vector3d(v(0), v(1), v(2));
    pose.orientation = UnitQuaternion(v(3), v(4), v(5), v(6));
    velocity.linear = Vector3d(v(7), v(8), v(9));
    velocity.angular = Vector3d(v(10), v(11), v(12));
    if (v.size() > 13)
    {
        int num_rotors = v.size() - 13;
        rotorspeed = std::vector<double>(num_rotors);
        for (int idx=0; idx < num_rotors; idx++)
        {
            rotorspeed[idx] = v(13 + idx);
        }
    }
    else
    {
        rotorspeed = std::vector<double>();
    }
}
VectorXd SimState_t::to_array()
{
    VectorXd res(13 + rotorspeed.size());
    res << pose.position,
        Vector4d(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()),
        velocity.linear,
        velocity.angular,
        VectorXd::Map(&rotorspeed[0],
                        rotorspeed.size());
    return res;
}
SimState_t SimState_t::strip_thrusters()
{
    auto res = SimState_t(*this);
    res.rotorspeed = std::vector<double>(0);
    return res;
}

Input::Input(const std::vector<double> &v) : value(num_input_channels, 0)
{
    if (v.size() > num_input_channels)
    {
        throw std::invalid_argument("Input vector longer than num_input_channels.");
    }
    std::copy(v.begin(), v.end(), value.begin());
}

//////////////////////////
// Define Airdata class
//////////////////////////

void Airdata::init_from_velocity(Vector3d velBody, Vector3d velWind)
{

    Vector3d relWind = velBody - velWind;
    double u = relWind.x();
    double v = relWind.y();
    double w = relWind.z();

    airspeed = sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));
    alpha = atan2(w, fabs(u));
    if (u == 0)
    {
        if (v == 0)
        {
            beta = 0;
        }
        else
        {
            beta = asin(v / abs(v));
        }
    }
    else
    {
        beta = atan2(v, u);
    }
}

// Wind is in Inertial Frame.
void Airdata::init_from_state_wind(SimState_t state, Vector3d wind)
{
    Vector3d u_b = state.velocity.linear;
    Quaterniond q_eb{state.pose.orientation.conjugate()};
    Vector3d u_w = q_eb * wind;
    init_from_velocity(u_b, u_w);
}

Eigen::Matrix3d Airdata::S_bw()
{
    Eigen::Matrix3d S;
    double sa{sin(alpha)};
    double ca{cos(alpha)};
    double sb{sin(beta)};
    double cb{cos(beta)};

    S << ca * cb, sb, sa * cb, -ca * sb, cb, -sa * sb, -sa, 0, ca;
    return S;
}

Eigen::Matrix3d Airdata::S_wb()
{
    return S_bw().transpose();
}

// Convert airdata to body-frame velocities
Vector3d Airdata::to_velocity()
{
    double u, v, w;
    u = airspeed * cos(alpha) * cos(beta);
    v = airspeed * sin(beta);
    w = airspeed * sin(alpha) * cos(beta);
    return Vector3d(u, v, w);
}

std::string Airdata::str()
{
    std::ostringstream oss;
    oss << std::setprecision(3)
        << "Airdata(Va="
        << airspeed
        << ", AoA[deg]="
        << rad_to_deg(alpha)
        << ", AoS[deg]="
        << rad_to_deg(beta)
        << ")";
    return oss.str();
}

////////////////////////////
// Kinematic Transformations
////////////////////////////

Vector3d getEulerDerivatives(Vector3d euler, Vector3d rates)
{
    double phi = euler(0);
    double theta = euler(1);
    double p = rates(0);
    double q = rates(1);
    double r = rates(2);

    double phiDot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
    double thetaDot = cos(phi) * q - sin(phi) * r;
    double psiDot = sin(phi) / cos(theta) * q + cos(phi) / cos(theta) * r;

    Vector3d result;
    result << phiDot, thetaDot, psiDot;
    return result;
}

Vector3d getAngularRatesFromEulerDerivatives(Vector3d euler, Vector3d eulerDot)
{
    double phi = euler(0);
    double theta = euler(1);
    double phiDot = eulerDot(0);
    double thetaDot = eulerDot(1);
    double psiDot = eulerDot(2);

    double pDot = phiDot - sin(theta) * psiDot;
    double qDot = cos(phi) * thetaDot + sin(phi) * cos(theta) * psiDot;
    double rDot = -sin(phi) * thetaDot + cos(phi) * cos(theta) * psiDot;

    Vector3d result(pDot, qDot, rDot);
    return result;
}

///////////////////
// Define PID class
///////////////////

// Constructor
PID::PID(double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
            double Tsi = 1.0 / 100, double Taui = 0.1)
{
    init();
    P = Pi;
    I = Ii;
    D = Di;
    satU = satUi;
    satL = satLi;
    trim = trimi;
    Ts = Tsi;
    Tau = Taui;
}

void PID::init(void)
{
    Iterm = 0;
    Iprev = 0;
    Eprev = 0;
    Uprev = 0;
    Dprev = 0;
}

double PID::step(double error)
{

    Iterm += I * Ts * error;
    double Dterm = 1.0 / (Tau + Ts) * (Tau * Dprev + error - Eprev);
    output = P * error + Iterm + D * Dterm + trim;

    //		double Pterm = P*error;
    //		Iterm += Ts/2 * (error + Eprev);
    //		double Dterm = (2*Tau - Ts)/(2*Tau + Ts) * Dprev + 2/(2*Tau + Ts)*(error - Eprev); //Bilinear transform, not working
    //		output = Pterm + I*Iterm + D*Dterm;

    if (output > satU)
    {
        output = satU;
        Iterm = Iprev;
    }
    if (output < satL)
    {
        output = satL;
        Iterm = Iprev;
    }
    Iprev = Iterm;
    Eprev = error;
    Uprev = output;
    Dprev = Dterm;
    return output;
}

double PID::step(double error, double dt)
{
    double Pterm = P * error;
    Iterm += I * error * dt;
    double Dterm = D / (D + dt / Tau) * (Uprev + (error - Eprev) / Tau);
    output = Pterm + Iterm + Dterm + trim;
    if (output > satU)
    {
        output = satU;
        Iterm = Iprev;
    }
    if (output < satL)
    {
        output = satL;
        Iterm = Iprev;
    }
    Iprev = Iterm;
    Eprev = error;
    Uprev = output;
    return output;
}

double PID::step(double error, double dt, double /*derivative*/)
{
    double Pterm = P * error;
    Iterm += I * error * dt;
    double Dterm = D / (D + dt / Tau) * (Uprev + (error - Eprev) / Tau);
    output = Pterm + Iterm + Dterm + trim;
    if (output > satU)
    {
        output = satU;
        Iterm = Iprev;
    }
    if (output < satL)
    {
        output = satL;
        Iterm = Iprev;
    }
    Iprev = Iterm;
    Uprev = output;
    return output;
}

// Destructor
PID::~PID()
{
}

///////////////////
// Define APID class
///////////////////

// Constructor
APID::APID(double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
            double Tsi = 1.0 / 100, double Taui = 0.1)
{
    init();
    Pinit = Pi;
    P = Pinit;
    Iinit = Ii;
    I = Iinit;
    Dinit = Di;
    D = Dinit;
    satU = satUi;
    satL = satLi;
    trim = trimi;
    Ts = Tsi;
    Tau = Taui;
}

void APID::init(void)
{
    Iterm = 0;
    Iprev = 0;
    Eprev = 0;
    Uprev = 0;
    Dprev = 0;
    Ierror = 0;
    bumplessI1 = 0;
    bumplessI2 = 0;
}

double APID::step(double error, bool track, double trInput)
{
    double PGainPrev = P;
    double IGainPrev = I;
    double DGainPrev = I;
    if (!track)
    {
        Ierror += error * Ts;
        // Ierror = std::max(-100.0, std::min(100.0, Ierror));

        P += (0.000001 * error * error - 0.01 * P) * Ts;
        I += (1e-7 * Ierror * Ierror - 0.5 * I) * Ts;
        // I += (0.00001*error*Ierror)*Ts;
        D += (1e-8 * pow((error - Eprev) / Ts, 2) - 0.05 * D) * Ts;
        if (I < Iinit)
        {
            I = Iinit;
        }
        bumplessI1 = 0;
        bumplessI2 = 0;
    }

    Iterm += I * Ts * error;
    output = P * error + Iterm + D * (error - Eprev) / Ts + trim;
    // output = P*error + Iterm + trim;

    if (!track)
    {
        if (output > satU)
        {
            output = satU;
            Iterm = Iprev;
            P = PGainPrev;
            I = IGainPrev;
            D = DGainPrev;
        }
        if (output < satL)
        {
            output = satL;
            Iterm = Iprev;
            P = PGainPrev;
            I = IGainPrev;
            D = DGainPrev;
        }
    }

    if (track)
    {
        trErr = 100 * (trInput - output);
        bumplessI1 += 25 * trErr * Ts;
        bumplessI2 += 2.0 * bumplessI1 * Ts;
        Iterm += (trErr + bumplessI1 + bumplessI2) * Ts;
        Ierror = 0;
        P = Pinit;
        I = Iinit;
        D = Dinit;
    }

    Iprev = Iterm;
    Eprev = error;
    return output;
}

// Destructor
APID::~APID()
{
}

/////////////////////////////
// WGS84 utility functions //
/////////////////////////////

double WGS84_RN(double lat)
{
    double sfi = sin(lat * M_PI / 180);
    return Geoid::WGS84_Ra / sqrt(1 - Geoid::WGS84_e2 * sfi * sfi);
}

double WGS84_RM(double lat)
{
    double sfi = sin(lat * M_PI / 180);
    return Geoid::WGS84_Ra * (1 - Geoid::WGS84_e2) / pow(1 - Geoid::WGS84_e2 * sfi * sfi, 1.5);
}

/////////////////////////////
// Other navigation functions
/////////////////////////////
std::pair<double, double> reproject(const double pos_north,
                                    const double pos_east,
                                    const double lat_home_deg,
                                    const double lon_home_deg)
{
    // reproject local position to gps coordinates
    const double x_rad = pos_north / WGS84_RN(lat_home_deg); // north
    const double y_rad = pos_east / WGS84_RM(lat_home_deg);	 // east
    const double c = sqrt(x_rad * x_rad + y_rad * y_rad);
    const double sin_c = sin(c);
    const double cos_c = cos(c);

    double lat_rad, lon_rad;

    double sinlat = sin(math_utils::deg_to_rad(lat_home_deg));
    double coslat = cos(math_utils::deg_to_rad(lat_home_deg));

    if (c != 0.0)
    {
        lat_rad = asin(cos_c * sinlat + (x_rad * sin_c * coslat) / c);
        lon_rad = (math_utils::deg_to_rad(lon_home_deg) + atan2(y_rad * sin_c, c * coslat * cos_c - x_rad * sinlat * sin_c));
    }
    else
    {
        lat_rad = math_utils::deg_to_rad(lat_home_deg);
        lon_rad = math_utils::deg_to_rad(lon_home_deg);
    }

    return std::make_pair(math_utils::rad_to_deg(lat_rad), math_utils::rad_to_deg(lon_rad));
}

//////////////////////////
// Miscellaneous Utilities
//////////////////////////

//////////////////////////////
// PPM and PWM functionalities

double PwmToHalfRange(uint16_t pwmValue)
// Convert a 1000-2000 us value to 0-1 range
{
    return ((double)pwmValue - 1000) / 1000;
}

double PwmToFullRange(uint16_t pwmValue)
// Convert a 1000-2000 us value to -1-1 range
{
    return ((double)pwmValue - 1500) / 500;
}

uint16_t HalfRangeToPwm(double signal)
// Convert a 0-1 range to 1000-2000 us range
{
    return (uint16_t)(signal * 1000 + 1000);
}

uint16_t FullRangeToPwm(double signal)
// Convert a -1-1 range to 1000-2000 us range
{
    return (uint16_t)(signal * 500 + 1500);
}

} // namespace uav_utils
} // namespace last_letter_lib
