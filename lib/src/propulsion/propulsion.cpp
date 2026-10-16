#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include <limits>

#include "last_letter_lib/logging.hpp"
#include "logging/log_types.hpp"

using namespace std;
using namespace last_letter_lib;
using namespace last_letter_lib::systems;

using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib {
namespace propulsion {

//////////////////////////////
// Thruster interfrace class
//////////////////////////////

// Constructor
Thruster::Thruster(string name)
    : Component(name),
    DynamicSystem()
{
    theta = 0; // Initialize propeller angle
    inputMotor = 0.0;
}

void Thruster::initialize_parameters()
{
    Component::initialize_parameters();

    set_param<double>("rotationDir",        1, false);
    set_param<int>("chanMotor", 0, false);
    set_param<int>("motorType", 0, false);
    set_param<double>("thrustMax", 20, false);
    set_param<double>("thrustMin", 0, false);
    set_param<double>("torqueMax", 1, false);
    set_param<double>("torqueMin", 0, false);
    set_param<double>("pwm_min", 1000, false);
    set_param<double>("pwm_max", 2000, false);
    set_param<double>("slew_max", 0.0, false);
    set_param<int>("momentum_drag/type", 0, false);
    set_param<double>("momentum_drag/coeff", 0.0, false);
    set_param<double>("momentum_drag/disc_area", 0.0, false);
    set_param<bool>("momentum_drag/axial", false, false);
    set_param<bool>("momentum_drag/lateral", true, false);

    set_param<vector<double>>("sys_x_0", x_0, false);
    set_param<vector<double>>("sys_u_0", u_0 , false);
}

void Thruster::update_parameters()
{
    Component::update_parameters();

    rotationDir = get_param<double>("rotationDir");
    chanMotor = get_param<int>("chanMotor");
    thrustMax = get_param<double>("thrustMax");
    thrustMin = get_param<double>("thrustMin");
    torqueMax = get_param<double>("torqueMax");
    torqueMin = get_param<double>("torqueMin");
    pwm_min = get_param<double>("pwm_min");
    pwm_max = get_param<double>("pwm_max");
    slew_max = get_param<double>("slew_max");
    if (!(pwm_max > pwm_min))
    {
        throw runtime_error("propulsion.cpp: pwm_max must be above pwm_min");
    }
    momentum_drag_type = get_param<int>("momentum_drag/type");
    momentum_drag_coeff = get_param<double>("momentum_drag/coeff");
    momentum_drag_disc_area = get_param<double>("momentum_drag/disc_area");
    momentum_drag_axial = get_param<bool>("momentum_drag/axial");
    momentum_drag_lateral = get_param<bool>("momentum_drag/lateral");
    if (momentum_drag_type < 0 || momentum_drag_type > 2)
    {
        throw runtime_error("propulsion.cpp: momentum_drag/type must be 0, 1 or 2");
    }

    x_0 = get_param<vector<double>>("sys_x_0");
    u_0 = get_param<vector<double>>("sys_u_0");
    if (!initialized) {
        reset(); // Set the correct dynamic system state dimensions.
        initialized = true;
    }
}

void Thruster::setInput(const Input &input)
{
    // chanMotor is optional; -1 means this thruster is not wired to a channel.
    if (chanMotor > -1)
    {
        set_throttle(input.value[chanMotor]);
    }
}

void Thruster::setInputPwm(const InputPwm_t &p_input)
{
    if (chanMotor > -1)
    {
        set_throttle(pwm_to_throttle(p_input.value[chanMotor]));
    }
}

double Thruster::pwm_to_throttle(double pwm_us) const
{
    return math_utils::constrain((pwm_us - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0);
}

stateType Thruster::dynamics(const stateType /*x*/, const stateType /*u*/, const double /*t*/)
{
    stateType dxdt;
    return dxdt; // Return an empty derivative vector.
}

stateType Thruster::outputs(const stateType /*x*/, const stateType u, const double /*t*/)
{
    return u; // Pass-through input.
}

// Engine physics step, container for the generic class
void Thruster::calc_model_impl()
{
    Component::calc_model_impl();

    relativeWind = local_state.velocity.linear - local_environment.wind;
    normalWind = relativeWind.x();
    if (!std::isfinite(normalWind))
    {
        throw runtime_error("propulsion.cpp: airspeed is not finite");
    }
    // This is the only write of inputMotor. The slew limit is per step, so it
    // lives here and not in set_throttle(). Without a limit the command passes.
    const double step_max = slew_max > 0.0 ? slew_max * dt : std::numeric_limits<double>::infinity();
    const double input_low = inputMotor - step_max;
    const double input_high = inputMotor + step_max;
    inputMotor = math_utils::constrain(input_commanded_, input_low, input_high);
    pre_propagation(local_state, local_environment);
    step_dynamics(u, dt);
    post_propagation();
    rotateProp();
    calc_wrench(local_state, local_environment);
    apply_momentum_drag();
}

void Thruster::apply_momentum_drag()
{
    const double thrust = std::fabs(wrench_sum.wrenchProp.force.x());
    const double lateral = momentum_drag_lateral ? 1.0 : 0.0;
    const Vector3d mask(momentum_drag_axial ? 1.0 : 0.0, lateral, lateral);
    // Gate the axial/lateral drag components by nullifying the equivalent wind components.
    const Vector3d wind = relativeWind.cwiseProduct(mask);

    double scale = 0.0;
    switch (momentum_drag_type)
    {
    case 1:
        scale = momentum_drag_coeff * thrust;
        break;
    case 2:
        scale = momentum_drag_coeff * std::sqrt(local_environment.density * momentum_drag_disc_area * thrust);
        break;
    default:
        break;
    }
    momentum_drag_ = -scale * wind;
    wrench_sum.wrenchProp.force += momentum_drag_;
}

void Thruster::register_log_channels()
{
    Component::register_log_channels(); // First let Component register basic data.

    auto channel = logging::get_channel(get_name());
    channel.register_value("omega", &omega);            // motor speed [rad/s]
    channel.register_value("theta", &theta);            // propeller angle [rad]
    channel.register_value("input_motor", &inputMotor); // control input [0-1]
    channel.register_value("input_commanded", &input_commanded_); // command before the slew limit
    channel.register_value("momentum_drag", &momentum_drag_);     // momentum drag force [N]
}

void Thruster::rotateProp() // Update propeller angle
{
    if (!std::isfinite(omega))
    {
        throw runtime_error("propulsion.cpp: non-finite omega value");
    }
    theta += omega * dt;
    if (theta > 2.0 * M_PI)
        theta -= 2 * M_PI;
    if (theta < 0.0)
        theta += 2 * M_PI;
}

std::unique_ptr<Thruster> buildThruster(ParameterManager propConfig)
{
    int motorType;
    motorType = propConfig.get<double>("motorType");
    string engineName = propConfig.get<string>("name");
    std::cout << "building engine model: ";
    std::unique_ptr<Thruster> engine;
    switch (motorType)
    {
    case 0:
        std::cout << "selecting no engine" << std::endl;
        engine = std::make_unique<NoEngine>(engineName);
        break;
    case 1:
        std::cout << "selecting Beard engine" << std::endl;
        engine = std::make_unique<EngBeard>(engineName);
        break;
    case 2:
        std::cout << "selecting piston engine" << std::endl;
        engine = std::make_unique<PistonEng>(engineName);
        break;
    case 3:
        std::cout << "selecting electric engine" << std::endl;
        engine = std::make_unique<ElectricEng>(engineName);
        break;
    case 4:
        std::cout << "selecting omega-controlled engine" << std::endl;
        engine = std::make_unique<EngOmegaControl>(engineName);
        break;
    case 5:
        std::cout << "selecting electric engine 2" << std::endl;
        engine = std::make_unique<ElectricEng2>(engineName);
        break;
    case 6:
        std::cout << "selecting simple thruster" << std::endl;
        engine = std::make_unique<ThrusterSimple>(engineName);
        break;
    default:
        throw runtime_error("Error while constructing motor");
        break;
    }
    engine->initialize(propConfig);
    return engine;
}

// Calculate nominal propeller pitch given its blade pitch angle
//
// Blade angle is given at 75% blade station.
//
// INPUTS:
//     diameter    Propeller diameter, m
//     beta        Blade pitch angle, rad
double pitch_from_pitch_angle(double diameter, double beta) {
    double r_nom = 0.75 * diameter / 2;  // Pitch angle is given at 75% of nominal radius
    return 2 * M_PI * r_nom * tan(beta);
}

// Calculate nominal propeller pitch angel given its pitch.
//
// Blade angle is given at 75% blade station.
//
// INPUTS:
//     diameter    Propeller diameter, m
//     pitch       Propeller pitch, m
double pitch_angle_from_pitch(double diameter, double pitch) {
   double r_nom = 0.75 * diameter / 2;  // Pitch angle is given at 75% of nominal radius
    return atan(pitch / (2 * M_PI * r_nom));
}

// Recalculates effective pitch when using non-standard hub_length.
//
// Applicable to propellers with foldable or removable blades.
//
// INPUTS:
//     pitch       The nominal propeller pitch, m
//     diam_nom    The nominal propeller diameter, m
//     diam_actual The actual propeller diameter, m
double correct_pitch_for_hub(double pitch, double diam_nom, double diam_actual) {
    double angle = pitch_angle_from_pitch(diam_nom, pitch);
    double pitch_actual = pitch_from_pitch_angle(diam_actual, angle);
    return pitch_actual;
}

//////////////////////////////
// Propeller class
//////////////////////////////

void Propeller::initialize_parameters()
{
    set_param<double>("diameter", 0.3048, false);
}

void Propeller::update_parameters()
{
    diameter = get_param<double>("diameter");
}

double Propeller::max_drag(double V) {
    double c_d = 1.17;  // Flat plate drag coefficient from https://en.wikipedia.org/wiki/Drag_coefficient
    double rho = 1.225;
    double chord = diameter / 10;  // rough number for chord
    double S = diameter * chord;
    return 0.5 * rho * pow(V,2) * S * c_d;
}

double Propeller::calc_advance_ratio(double V, double n) {
    if (fabs(n) != 0) {
        return V / (n * diameter);
    } else if (V == 0){
            return 0;
    } else {
        return math_utils::sign(V) * std::numeric_limits<float>::infinity();
    }
}

double Propeller::calc_thrust(double V, double n, double rho) {
    double ar = calc_advance_ratio(V, n);
    double Ct = calc_coeff_thrust(ar);

    double thrust = Ct * rho * pow(n,2) * pow(diameter, 4);
    // Cap the thrust of the propeller if negative
    if (thrust < 0) {
        thrust = max(thrust, -max_drag(V));
    }
    return thrust;
}

double Propeller::calc_power(double V, double n, double rho) {
    double ar = calc_advance_ratio(V, n);
    double Cp = calc_coeff_power(ar);

    double power = Cp * rho * pow(n,3) * pow(diameter, 5);
    double thrust = calc_thrust(V, n, rho);
    // Don't perform power calculations if thrust is negative.
    // Because the propeller is outside its operational envelope.
    if (thrust < 0) {
        power = 0;
    }
    return power;
}

double Propeller::calc_torque(double V, double n, double rho) {
    double ar = calc_advance_ratio(V, n);
    double c_p = calc_coeff_power(ar);
    double thrust = calc_thrust(V, n, rho);
    // Don't perform power calculations if thrust is negative.
    // Because the propeller is outside its operational envelope.
    double c_q = 0;
    if (thrust > 0) {
        c_q = c_p / (2 * M_PI);
    } else {
        c_q = 0;
    }
    return c_q * rho * pow(n, 2) * pow(diameter, 5);
}

double Propeller::calc_efficiency(double ar) {
    double Ct = calc_coeff_thrust(ar);
    double Cp = calc_coeff_power(ar);
    return Ct * ar / Cp;
}

uav_utils::Wrench_t Propeller::calc_wrench(double V, double n, double rho) {
    double f_x = calc_thrust(V, n, rho);
    double t_x = calc_torque(V, n, rho);
    auto force = Vector3d(f_x, 0, 0);
    auto torque = Vector3d(t_x, 0, 0);
    return uav_utils::Wrench_t(force, torque);
}

//////////////////////////////
// PropellerStandard class
//////////////////////////////

void PropellerStandard::initialize_parameters() {
    Propeller::initialize_parameters();

    set_param<double>("pitch", 0, false);

    set_param<double>("c_thrust/polyType", 0, false);
    set_param<double>("c_thrust/polyNo", 3, false);
    std::vector<double> c_thrust_coeffs = {0.064295, -0.045845, -0.161243};
    set_param<vector<double>>("c_thrust/coeffs", c_thrust_coeffs, false);

    set_param<double>("c_power/polyType", 0, false);
    set_param<double>("c_power/polyNo", 3, false);
    std::vector<double> c_power_coeffs = {0.018441, 0.016194, -0.084206};
    set_param<vector<double>>("c_power/coeffs", c_power_coeffs, false);
}

void PropellerStandard::update_parameters() {
    Propeller::update_parameters();

    pitch = get_param<double>("pitch");

    // Create thrust polynomial
    ParameterManager c_thrust_poly_config = params_.filter("c_thrust/");
    c_thrust = buildPolynomial(c_thrust_poly_config);

    // Create power polynomial
    ParameterManager c_power_poly_config = params_.filter("c_power/");
    c_power = buildPolynomial(c_power_poly_config);
}

double PropellerStandard::calc_coeff_thrust(double ar) {
    double c_t;
    if (isfinite(ar)) {
        c_t = c_thrust->evaluate(ar);
    } else {  // This applies in omega=0 situations and is a numerical hack
        c_t = c_thrust->coeffs[0];
    }
    return c_t;
}

double PropellerStandard::calc_coeff_power(double ar) {
    double c_t;
    if (isfinite(ar)) {
        c_t = c_power->evaluate(ar);
    } else {  // This applies in omega=0 situations and is a numerical hack
        c_t = c_power->coeffs[0];
    }
    return c_t;
}

Propeller *buildPropeller(ParameterManager propConfig)
{
    int propellerType;
    propellerType = propConfig.get<double>("propellerType");
    string propellerName = propConfig.get<string>("name");
    std::cout << "building propeller model: ";
    Propeller *propeller;
    switch (propellerType)
    {
    case 0:
        std::cout << "selecting PropellerStandard" << std::endl;
        propeller = new PropellerStandard(propellerName);
        break;
    default:
        throw runtime_error("Error while constructing propeller");
        break;
    }
    propeller->initialize(propConfig);
    return propeller;
}

} // namespace propulsion
} // namespace last_letter_lib
