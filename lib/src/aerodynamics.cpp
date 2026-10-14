#include <stdexcept>
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/math_utils.hpp"

#include "last_letter_lib/logging.hpp"
#include "logging/log_types.hpp"

using namespace std;
using namespace last_letter_lib;
using last_letter_lib::programming_utils::ParameterManager;
using last_letter_lib::math_utils::rad_to_deg;

using Eigen::Quaterniond;

namespace last_letter_lib {
namespace aerodynamics {

// Calculate dynamic pressure.
// INPUTS:
// rho: air density, kg/m^3
// v_a: airspeed, m/s
// OUTPUTS:
// dynamic pressure, Pa
double calc_dynamic_pressure(double rho, double v_a)
{
    return 0.5 * rho * v_a * v_a;
}

// Calculate the required bank angle to sustain a turn.
// Source: Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice. Eq. 5.16.
//
// INPUTS:
//     R       The turn radius, m
//     v       The ground speed, m/s
//     gamma   The flight path angle, rad
//     g       Gravity acceleration, m/s^2
//
// Assumes a coordinated turn.
double calc_bank_from_radius(double R, double v, double gamma, double g)
{
    return atan2(v*v * cos(gamma), R * g);
}

//////////////////////////
// Define Aerodynamics class
//////////////////////////

void Aerodynamics::initialize_parameters()
{
    Component::initialize_parameters();

    set_param<double>("chanAileron", -1, false);
    set_param<double>("chanElevator", -1, false);
    set_param<double>("chanRudder", -1, false);
    set_param<double>("deltaa_max", deg_to_rad(20), false);
    set_param<double>("deltae_max", deg_to_rad(20), false);
    set_param<double>("deltar_max", deg_to_rad(20), false);
}

void Aerodynamics::update_parameters()
{
    Component::update_parameters();

    chanAileron = get_param<int>("chanAileron");
    chanElevator = get_param<int>("chanElevator");
    chanRudder = get_param<int>("chanRudder");
    deltaa_max = get_param<double>("deltaa_max");
    deltae_max = get_param<double>("deltae_max");
    deltar_max = get_param<double>("deltar_max");
}

void Aerodynamics::setInput(const Input &input)
{
    // The setters convert the -1 - 1 input range to radians.
    if (chanAileron > -1) { set_aileron(input.value[chanAileron]); }
    if (chanElevator > -1) { set_elevator(input.value[chanElevator]); }
    if (chanRudder > -1) { set_rudder(input.value[chanRudder]); }
}

void Aerodynamics::setInputPwm(const InputPwm_t &p_input)
{
    // Convert PPM to the -1 - 1 range the setters expect.
    if (chanAileron > -1) { set_aileron(PwmToFullRange(p_input.value[chanAileron])); }
    if (chanElevator > -1) { set_elevator(PwmToFullRange(p_input.value[chanElevator])); }
    if (chanRudder > -1) { set_rudder(PwmToFullRange(p_input.value[chanRudder])); }
}

// One step in the physics engine
void Aerodynamics::calc_model_impl()
{
    // Perform gravity calculations.
    Component::calc_model_impl();

    p = local_state.velocity.angular(0);
    q = local_state.velocity.angular(1);
    r = local_state.velocity.angular(2);

    airdata.init_from_velocity(local_state.velocity.linear, local_environment.wind);
    // Calculate the new, relative air data
    airdata.airspeed = airdata.airspeed;
    airdata.alpha = airdata.alpha;
    airdata.beta = airdata.beta;

}

/////////////////////////////
// Define StdLinearAero class
/////////////////////////////

void StdLinearAero::initialize_parameters()
{
    Aerodynamics::initialize_parameters();

    set_param<double>("c", 0.24, false);
    set_param<double>("b", 1.88, false);
    set_param<double>("s", 0.45, false);
    set_param<double>("c_L_0", 0.56, false);
    set_param<double>("c_L_alpha", 6.9, false);
    set_param<double>("c_L_beta", 0, false);
    set_param<double>("c_L_qn", 0, false);
    set_param<double>("c_L_deltae", 0, false);
    set_param<double>("c_D_qn", 0, false);
    set_param<double>("c_D_0", 0.09, false);
    set_param<double>("c_D_deltae", 0, false);
    set_param<double>("c_Y_0", 0, false);
    set_param<double>("c_Y_beta", -0.98, false);
    set_param<double>("c_Y_pn", 0, false);
    set_param<double>("c_Y_rn", 0, false);
    set_param<double>("c_Y_deltaa", 0, false);
    set_param<double>("c_Y_deltar", -0.2, false);
    set_param<double>("c_l_0", 0, false);
    set_param<double>("c_l_beta", -0.12, false);
    set_param<double>("c_l_pn", -1.0, false);
    set_param<double>("c_l_rn", 0.14, false);
    set_param<double>("c_l_deltaa", 0.25, false);
    set_param<double>("c_l_deltar", -0.037, false);
    set_param<double>("c_m_0", 0.045, false);
    set_param<double>("c_m_alpha", -0.7, false);
    set_param<double>("c_m_qn", -20.0, false);
    set_param<double>("c_m_deltae", 1.0, false);
    set_param<double>("c_n_0", 0, false);
    set_param<double>("c_n_beta", 0.25, false);
    set_param<double>("c_n_pn", 0.022, false);
    set_param<double>("c_n_rn", -1.0, false);
    set_param<double>("c_n_deltaa", 0, false);
    set_param<double>("c_n_deltar", 0.1, false);
    set_param<double>("oswald", 0.9, false);
    set_param<double>("mcoeff", 50, false);
    set_param<double>("alpha_stall", 0.4712, false);
}

void StdLinearAero::update_parameters()
{
    Aerodynamics::update_parameters();

    // Read aerodynamic coefficients from parameter server
    c = get_param<double>("c");
    b = get_param<double>("b");
    s = get_param<double>("s");
    c_lift_0 = get_param<double>("c_L_0");
    c_lift_a0 = get_param<double>("c_L_alpha");
    c_lift_q = get_param<double>("c_L_qn");
    c_lift_deltae = get_param<double>("c_L_deltae");
    c_drag_q = get_param<double>("c_D_qn");
    c_drag_p = get_param<double>("c_D_0");
    c_drag_deltae = get_param<double>("c_D_deltae");
    c_y_0 = get_param<double>("c_Y_0");
    c_y_b = get_param<double>("c_Y_beta");
    c_y_p = get_param<double>("c_Y_pn");
    c_y_r = get_param<double>("c_Y_rn");
    c_y_deltaa = get_param<double>("c_Y_deltaa");
    c_y_deltar = get_param<double>("c_Y_deltar");
    c_l_0 = get_param<double>("c_l_0");
    c_l_b = get_param<double>("c_l_beta");
    c_l_p = get_param<double>("c_l_pn");
    c_l_r = get_param<double>("c_l_rn");
    c_l_deltaa = get_param<double>("c_l_deltaa");
    c_l_deltar = get_param<double>("c_l_deltar");
    c_m_0 = get_param<double>("c_m_0");
    c_m_a = get_param<double>("c_m_alpha");
    c_m_q = get_param<double>("c_m_qn");
    c_m_deltae = get_param<double>("c_m_deltae");
    c_n_0 = get_param<double>("c_n_0");
    c_n_b = get_param<double>("c_n_beta");
    c_n_p = get_param<double>("c_n_pn");
    c_n_r = get_param<double>("c_n_rn");
    c_n_deltaa = get_param<double>("c_n_deltaa");
    c_n_deltar = get_param<double>("c_n_deltar");
    oswald = get_param<double>("oswald");
    M = get_param<double>("mcoeff");
    alpha0 = get_param<double>("alpha_stall");
}

void Aerodynamics::register_log_channels()
{
    Component::register_log_channels(); // First let Component register basic data.
    logging::get_channel(get_name()).register_value("airdata", &airdata);
}

void StdLinearAero::calc_model_impl()
{
    Aerodynamics::calc_model_impl();

    // Read air density
    rho = local_environment.density;

    // request lift and drag alpha-coefficients from the corresponding functions
    double c_lift_a = liftCoeff(airdata.alpha);
    double c_drag_a = dragCoeff(airdata.alpha, airdata.beta);

    // Rotate coefficients from wind to body frame.
    const Vector3d c_alpha_body = airdata.S_wb() * Vector3d(-c_drag_a, 0, -c_lift_a);
    const Vector3d c_q_body = airdata.S_wb() * Vector3d(-c_drag_q, 0, -c_lift_q);
    const double c_x_a = c_alpha_body.x();
    const double c_z_a = c_alpha_body.z();
    const double c_x_q = c_q_body.x();
    const double c_z_q = c_q_body.z();

    // calculate aerodynamic force
    double qbar = calc_dynamic_pressure(rho, airdata.airspeed) * s; // Calculate dynamic pressure
    double ax, ay, az;
    if (airdata.airspeed == 0)
    {
        ax = 0;
        ay = 0;
        az = 0;
    }
    else
    {
        ax = qbar * (c_x_a + c_x_q * c * q / (2 * airdata.airspeed) - c_drag_deltae * cos(airdata.alpha) * fabs(inputElevator) + c_lift_deltae * sin(airdata.alpha) * inputElevator);
        // split c_x_deltae to include "abs" term
        ay = qbar * (c_y_0 + c_y_b * airdata.beta + c_y_p * b * p / (2 * airdata.airspeed) + c_y_r * b * r / (2 * airdata.airspeed) + c_y_deltaa * inputAileron + c_y_deltar * inputRudder);
        az = qbar * (c_z_a + c_z_q * c * q / (2 * airdata.airspeed) - c_drag_deltae * sin(airdata.alpha) * fabs(inputElevator) - c_lift_deltae * cos(airdata.alpha) * inputElevator);
        // split c_z_deltae to include "abs" term
    }

    wrench_sum.wrenchAero.force = Vector3d(ax, ay, az);

    // calculate aerodynamic torque
    double la, na, ma;
    if (airdata.airspeed == 0)
    {
        la = 0;
        ma = 0;
        na = 0;
    }
    else
    {
        la = qbar * b * (c_l_0 + c_l_b * airdata.beta + c_l_p * b * p / (2 * airdata.airspeed) + c_l_r * b * r / (2 * airdata.airspeed) + c_l_deltaa * inputAileron + c_l_deltar * inputRudder);
        ma = qbar * c * (c_m_0 + c_m_a * airdata.alpha + c_m_q * c * q / (2 * airdata.airspeed) + c_m_deltae * inputElevator);
        na = qbar * b * (c_n_0 + c_n_b * airdata.beta + c_n_p * b * p / (2 * airdata.airspeed) + c_n_r * b * r / (2 * airdata.airspeed) + c_n_deltaa * inputAileron + c_n_deltar * inputRudder);
    }

    wrench_sum.wrenchAero.torque = Vector3d(la, ma, na);
}

//////////////////////////
// C_lift_alpha calculation
double StdLinearAero::liftCoeff(double alpha)
{
    double sigmoid = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) / (1 + exp(-M * (alpha - alpha0))) / (1 + exp(M * (alpha + alpha0)));
    double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha);						 // Lift at small AoA
    double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); // Lift beyond stall

    // std::cout << c_lift_0 << " " << c_lift_a0 << std::endl;

    double result = linear + flatPlate;
    return result;
}

//////////////////////////
// C_drag_alpha calculation

double StdLinearAero::_dragCoeff(double alpha, double /*beta*/)
{
    AR = pow(b, 2) / s;
    double c_drag_a = c_drag_p + pow(c_lift_0 + c_lift_a0 * alpha, 2) / (M_PI * oswald * AR);

    return c_drag_a;
}

double StdLinearAero::dragCoeff(double alpha, double beta)
{
    double sigmoid = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) / (1 + exp(-M * (alpha - alpha0))) / (1 + exp(M * (alpha + alpha0)));
    const double linear = (1.0 - sigmoid) * _dragCoeff(alpha, beta);
    double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 3)); // Drag beyond stall
    const double c_drag = linear + flatPlate;

    double flying_backwards_sign{1};
    if (fabs(beta) > M_PI / 2) {
        flying_backwards_sign = -1;
    }

    return c_drag * flying_backwards_sign;
}

/////////////////////////
// Define PolynomialAero class
/////////////////////////

void PolynomialAero::initialize_parameters()
{
    StdLinearAero::initialize_parameters();

    // Create CLift polynomial
    set_param<double>("cLiftPoly/polyType", 0, false);
    set_param<double>("cLiftPoly/polyNo", 1, false);
    std::vector<double> cLiftPolyCoeffs = {0.56, 6.9};
    set_param<vector<double>>("cLiftPoly/coeffs", cLiftPolyCoeffs, false);

    // Create CDrag polynomial
    set_param<double>("cDragPoly/polyType", 0, false);
    set_param<double>("cDragPoly/polyNo", 1, false);
    std::vector<double> cDragPolyCoeffs = {0.09, 0.5};
    set_param<vector<double>>("cDragPoly/coeffs", cDragPolyCoeffs, false);
}

void PolynomialAero::update_parameters()
{
    StdLinearAero::update_parameters();

    ParameterManager liftPolyConfig = params_.filter("cLiftPoly");
    liftCoeffPoly = programming_utils::buildPolynomial(liftPolyConfig);
    // Create CDrag polynomial
    ParameterManager dragPolyConfig = params_.filter("cDragPoly");
    dragCoeffPoly = programming_utils::buildPolynomial(dragPolyConfig);
}

//////////////////////////
// C_lift_alpha calculation
double PolynomialAero::liftCoeff(double alpha)
{
    return liftCoeffPoly->evaluate(alpha);
}

//////////////////////////
// C_drag_airdata.alpha calculation
double PolynomialAero::_dragCoeff(double alpha, double /*beta*/)
{
    return dragCoeffPoly->evaluate(alpha);
}

//////////////////////////
// Define SimpleDrag class
//////////////////////////

void SimpleDrag::initialize_parameters()
{
    StdLinearAero::initialize_parameters();

    set_param<double>("c_D_alpha", 0.5, false);
}

void SimpleDrag::update_parameters()
{
    StdLinearAero::update_parameters();
    c_drag_a = get_param<double>("c_D_alpha");
}

double SimpleDrag::_dragCoeff(double alpha, double /*beta*/)
{
    return c_drag_p + c_drag_a * fabs(alpha);
}

/////////////////////////////
// Define BluffBody class
/////////////////////////////

void BluffBody::initialize_parameters()
{
    Aerodynamics::initialize_parameters();

    set_param<vector<double>>("area_cd", {0.0, 0.0, 0.0}, false);
    set_param<vector<double>>("rate_damping", {0.0, 0.0, 0.0}, false);
}

void BluffBody::update_parameters()
{
    Aerodynamics::update_parameters();

    area_cd = get_param<vector<double>>("area_cd");
    rate_damping = get_param<vector<double>>("rate_damping");
    if (area_cd.size() != 3)
    {
        throw runtime_error("aerodynamics.cpp: BluffBody area_cd must have 3 elements");
    }
    if (rate_damping.size() != 3)
    {
        throw runtime_error("aerodynamics.cpp: BluffBody rate_damping must have 3 elements");
    }
}

void BluffBody::calc_model_impl()
{
    // Gravity and airdata first.
    Aerodynamics::calc_model_impl();

    const double rho = local_environment.density;
    const Vector3d v = local_state.velocity.linear - local_environment.wind;
    const Vector3d w = local_state.velocity.angular;
    Vector3d force;
    Vector3d torque;
    for (int i = 0; i < 3; i++)
    {
        force(i) = -0.5 * rho * area_cd[i] * std::fabs(v(i)) * v(i);
        torque(i) = -rate_damping[i] * w(i);
    }
    wrench_sum.wrenchAero.force = force;
    wrench_sum.wrenchAero.torque = torque;
    if (!wrench_sum.wrenchAero.force.allFinite() || !wrench_sum.wrenchAero.torque.allFinite())
    {
        throw runtime_error("aerodynamics.cpp: BluffBody wrench is not finite");
    }
}

// Build aerodynamics model
std::unique_ptr<Aerodynamics> buildAerodynamics(ParameterManager config)
{
    const int aerodynamicsType = config.get<double>("aerodynamicsType");
    const string aerodynamics_name = config.get<string>("name");

    std::unique_ptr<Aerodynamics> aerodynamics;
    std::cout << "building aerodynamics model type " << aerodynamicsType << ":\n";
    switch (aerodynamicsType)
    {
    case 0:
        std::cout << "selecting no aerodynamics model" << std::endl;
        aerodynamics = std::make_unique<NoAerodynamics>(aerodynamics_name);
        break;
    case 1:
        std::cout << "selecting StdLinearAero aerodynamics" << std::endl;
        aerodynamics = std::make_unique<StdLinearAero>(aerodynamics_name);
        break;
    case 2:
        std::cout << "selecting HCUAVAero aerodynamics" << std::endl;
        aerodynamics = std::make_unique<PolynomialAero>(aerodynamics_name);
        break;
    case 3:
        std::cout << "selecting simplified drag aerodynamics" << std::endl;
        aerodynamics = std::make_unique<SimpleDrag>(aerodynamics_name);
        break;
    case 4:
        std::cout << "selecting bluff body aerodynamics" << std::endl;
        aerodynamics = std::make_unique<BluffBody>(aerodynamics_name);
        break;
    default:
        stringstream ss;
        ss << "Error while constructing aerodynamics: Unknown type " << aerodynamicsType;
        throw runtime_error(ss.str());
        break;
    }
    aerodynamics->initialize(config);
    return aerodynamics;
}

} // namespace aerodynamics
} // namespace last_letter_lib
