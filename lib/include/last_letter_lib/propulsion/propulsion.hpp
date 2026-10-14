///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#pragma once

#include <memory>

#include <Eigen/Eigen>
#include <string>

#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/systems.hpp"

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////

using Eigen::Vector3d;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib::systems;
using last_letter_lib::math_utils::Polynomial;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib
{
namespace propulsion
{

class Thruster : public Component, public DynamicSystem
{
public:
    ///////////
    // Variables
    // dt is inherited from Component.
    double inputMotor;  // control input, constrained to (-1 to 1)
    int chanMotor;
    double omega{0};       // motor angular speed in rad/s
    double rotationDir; // motor direction of rotation
    double theta;       // propeller angle in rads
    double normalWind;  // scalar wind normal to propeller disc
    double thrustMax, thrustMin;
    double torqueMax, torqueMin;
    double pwm_min{1000}, pwm_max{2000}; // PWM in us at zero and at full command
    double slew_max{0.0};                // Command slew limit in 1/s, 0 disables it
    int momentum_drag_type{0};           // 0 none, 1 linear, 2 disc
    double momentum_drag_coeff{0.0};     // s/m for type 1, the ArduPilot mdrag for type 2
    double momentum_drag_disc_area{0.0}; // m2, type 2 only
    bool momentum_drag_axial{false};     // Apply to the x component of relativeWind
    bool momentum_drag_lateral{true};    // Apply to the y and z components
    Vector3d relativeWind;
    // stateType u holds the system input.
    // stateType x holds the system state.

    ///////////
    // Functions
    Thruster(string name);
    virtual ~Thruster() {};

    void initialize_parameters() override;
    virtual void update_parameters() override;

    // Store the throttle directly, bypassing the channel lookup. Both setters
    // below funnel through this. Only calc_model_impl() writes inputMotor, once
    // per step, so a new command takes effect at the next step.
    void set_throttle(double throttle) { input_commanded_ = math_utils::constrain(throttle, -1.0, 1.0); }
    void setInput(const Input &input);                                                              // store control input
    void setInputPwm(const InputPwm_t &input);                                                      // store PWM control input
    virtual stateType dynamics(const stateType x, const stateType u, const double t) override;
    virtual stateType outputs(const stateType x, const stateType u, const double t) override;
    void calc_model_impl() override;          // engine physics step, container for the generic class
    virtual void pre_propagation(SimState_t states, Environment_t environment) = 0; // Build the input u of the dynamic system.
    virtual void post_propagation() override = 0; // Distribute the propagated state into other local variables.
    void rotateProp();                                                                              // Update the propeller angle
    // Each override assigns every component of wrenchProp and never accumulates.
    // After it, Thruster adds the momentum drag through apply_momentum_drag().
    virtual void calc_wrench(SimState_t states, Environment_t environment) = 0;
    double torque_sign() {return rotationDir;}

protected:
    void register_log_channels() override; // adds omega/theta/input to base wrench_sum
    bool initialized{false};

private:
    // Add the momentum drag to wrenchProp.force, scaled by the thrust that
    // calc_wrench() produced.
    void apply_momentum_drag();

    double input_commanded_{0.0}; // Command before the slew limit, in -1 to 1
    Vector3d momentum_drag_{Vector3d::Zero()}; // Last momentum drag force, in the thruster frame
};

class NoEngine : public Thruster
{
public:
	NoEngine(string name);
	~NoEngine() {};

    void pre_propagation(SimState_t, Environment_t) override {};
    void post_propagation() override {};
	void calc_wrench(SimState_t, Environment_t) override
	{
		wrench_sum.wrenchProp.force = Vector3d::Zero();
		wrench_sum.wrenchProp.torque = Vector3d::Zero();
	}
};

class ThrusterSimple : public Thruster
{
public:
    ///////////
    //Functions
    ThrusterSimple(string name);
    ~ThrusterSimple() {};

    void pre_propagation(SimState_t, Environment_t) override {};
    void post_propagation() override {};
    void calc_wrench(SimState_t states, Environment_t environment) override;
};

class EngBeard : public Thruster
{
public:
    ///////////
    //Variables
    double s_prop, c_prop, k_motor, k_t_p, k_omega;
    double expo; // Thrust curve expo as in ArduPilot, 1 keeps the outflow linear in the input
    double rho;

    ///////////
    //Functions
    EngBeard(string name);
    ~EngBeard();
    void initialize_parameters() override
    {
        Thruster::initialize_parameters();

        set_param<double>("s_prop", 1.0, false);
        set_param<double>("c_prop", 0.33, false);
        set_param<double>("k_motor", 30, false);
        set_param<double>("k_t_p", 1e-6, false);
        set_param<double>("k_omega", 800, false);
        set_param<double>("expo", 1.0, false);
    }

    void update_parameters() override;

    void pre_propagation(SimState_t, Environment_t) override {};
    void post_propagation() override;
    void calc_wrench(SimState_t states, Environment_t environment) override;
};

class ElectricEng : public Thruster
{
public:
    ////////////////
    // Variables //
    ////////////////
    double omegaMin, omegaMax;
    double prop_diam, engInertia, rho;
    double Kv, Rm, I0;
    // Battery specification
    int Cells; // Number of LiPo cells
    double Rs; // Battery internal resistance
    // TODO: Find where the message needs to be introduced in the ROS wrapper
    // last_letter_msgs::ElectricEng message;
    // ros::Publisher pub;

    //////////////
    // Members //
    //////////////
    Polynomial *npPoly, *propPowerPoly;

    ////////////////
    // Functions //
    ////////////////
    ElectricEng(string name);
    ~ElectricEng();
    void initialize_parameters() override
    {
        Thruster::initialize_parameters();

        set_param<double>("prop_diam", 0.4064, false);
        set_param<double>("engInertia", 200e-6, false);
        set_param<double>("Kv", 8.17, false);
        set_param<double>("Rm", 0.1, false);
        set_param<double>("Rs", 0.003, false);
        set_param<double>("Cells", 6, false);
        set_param<double>("I0", 0.5, false);
        std::vector<double> radpslimits = {0.01, 1000};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        // TODO: Need realistic coefficients here.
        set_param<double>("nCoeffPoly/polyType", 0, false);
        set_param<double>("nCoeffPoly/polyNo", 2, false);
        std::vector<double> nCoeffPolyCoeffs = {0, 0.03, -0.0003};
        set_param<vector<double>>("nCoeffPoly/coeffs", nCoeffPolyCoeffs, false);

        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 3, false);
        std::vector<double> propPowerPolyCoeffs = {0.0738, -0.1329, 0.2872, -0.2853};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);
    }
    void update_parameters() override;

    void pre_propagation(SimState_t, Environment_t) override;
    void post_propagation() override;
    stateType dynamics(const stateType x, const stateType u, const double t) override;
    void calc_wrench(SimState_t states, Environment_t environment) override;
};

class ElectricEng2 : public Thruster
{
public:
    ////////////////
    // Variables //
    ////////////////
    double omegaMin, omegaMax;
    double prop_diam, engInertia, rho;
    double Kv, Rm, I0;
    // Battery specification
    int Cells; // Number of LiPo cells
    double Rs; // Battery internal resistance
    // TODO: Find where the message needs to be introduced in the ROS wrapper
    // last_letter_msgs::ElectricEng message;
    // ros::Publisher pub;

    //////////////
    // Members //
    //////////////
    Polynomial *propThrustPoly, *propPowerPoly;
    double propThrustMultiplier;

    ////////////////
    // Functions //
    ////////////////
    ElectricEng2(string name);
    ~ElectricEng2();
    void initialize_parameters() override
    {
        Thruster::initialize_parameters();

        set_param<double>("prop_diam", 0.4064, false);
        set_param<double>("engInertia", 200e-6, false);
        set_param<double>("Kv", 8.17, false);
        set_param<double>("Rm", 0.1, false);
        set_param<double>("Rs", 0.003, false);
        set_param<double>("Cells", 6, false);
        set_param<double>("I0", 0.5, false);
        std::vector<double> radpslimits = {0.01, 1000};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        set_param<double>("propThrustPoly/polyType", 0, false);
        set_param<double>("propThrustPoly/polyNo", 5, false);
        std::vector<double> propThrustPolyCoeffs = {0.0737, -0.4778, 2.2161, -5.5296, 6.2749, -2.6331};
        set_param<vector<double>>("propThrustPoly/coeffs", propThrustPolyCoeffs, false);

        set_param<double>("propThrustMultiplier", 1.0, false);
        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 3, false);
        std::vector<double> propPowerPolyCoeffs = {0.0738, -0.1329, 0.2872, -0.2853};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);

        // The linear law with these values is the momentum drag this class
        // always had.
        set_param<int>("momentum_drag/type", 1, false);
        set_param<double>("momentum_drag/coeff", 0.0047, false);
        set_param<bool>("momentum_drag/axial", false, false);
        set_param<bool>("momentum_drag/lateral", true, false);
    }
    void update_parameters() override;

    void pre_propagation(SimState_t, Environment_t) override;
    void post_propagation() override;
    stateType dynamics(const stateType x, const stateType u, const double t) override;
    void calc_wrench(SimState_t states, Environment_t environment) override;
};

class EngOmegaControl : public Thruster
{
public:
	///////////
	//Variables
	double prop_diam, omega_max;
	double airspeed, rho;
	double thrust, torque;
	Polynomial *power_poly, *thrust_poly;

	///////////
	//Functions
	EngOmegaControl(string name);
	~EngOmegaControl();
    void initialize_parameters() override
    {
        Thruster::initialize_parameters();

        set_param<double>("prop_diam", 0.28, false);
        set_param<double>("omega_max", 1050, false);

        set_param<double>("thrust_poly/polyType", 0, false);
        set_param<double>("thrust_poly/polyNo", 2, false);
        std::vector<double> thrust_polyCoeffs = {0.1133, -0.0740, -0.1849};
        set_param<vector<double>>("thrust_poly/coeffs", thrust_polyCoeffs, false);

        set_param<double>("power_poly/polyType", 0, false);
        set_param<double>("power_poly/polyNo", 2, false);
        std::vector<double> power_polyCoeffs = {0.0440, 0.0059, -0.0732};
        set_param<vector<double>>("power_poly/coeffs", power_polyCoeffs, false);
    }

    void update_parameters() override;

    void pre_propagation(SimState_t, Environment_t) override {};
    void post_propagation() override;
	void calc_wrench(SimState_t states, Environment_t environment) override;
};

class PistonEng : public Thruster
{
public:
	////////////
	// Variables
	double omegaMin, omegaMax;
	double prop_diam, engInertia, rho;

	//////////
	// Members
	Polynomial *engPowerPoly;
	Polynomial *npPoly;
	Polynomial *propPowerPoly;

	////////////
	// Functions
	PistonEng(string name);
    ~PistonEng();
    void initialize_parameters() override
    {
        Thruster::initialize_parameters();

        // TODO: These defaults have to be replaced with realistic numbers.
        set_param<double>("prop_diam", 0.5588, false);
        set_param<double>("engInertia", 0.01, false);
        std::vector<double> radpslimits = {100, 700};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        set_param<double>("engPowerPoly/polyType", 0, false);
        set_param<double>("engPowerPoly/polyNo", 3, false);
        std::vector<double> engPowerPolyCoeffs = {-0.329, 2.68e-4, 3.784e-7, -5.278e-11};
        set_param<vector<double>>("engPowerPoly/coeffs", engPowerPolyCoeffs, false);

        set_param<double>("propCoeffPoly/polyType", 0, false);
        set_param<double>("propCoeffPoly/polyNo", 2, false);
        std::vector<double> propCoeffPolyCoeffs = {0, 0.03, -0.0003};
        set_param<vector<double>>("propCoeffPoly/coeffs", propCoeffPolyCoeffs, false);

        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {0.0289, -0.0405};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);
    }
    void update_parameters() override;

    void pre_propagation(SimState_t, Environment_t) override;
    void post_propagation() override;
    stateType dynamics(const stateType x, const stateType u, const double t) override;
	void calc_wrench(SimState_t states, Environment_t environment) override;
};

// A propeller model.
//
// Its parameters are according to these equations:
//
// .. math::
//
//     J = V_a / (n D)
//
//     T = \\rho n^2 D^4 C_{thrust}(J)
//
//     P = \\rho n^3 D^5 C_{power}(J)
class Propeller : public Parametrized
{
public:
    double diameter;

    Propeller(string name) : Parametrized(name) {};

    void initialize_parameters() override;
    void update_parameters() override;

    // Calculate the advance ratio of a propeller
    //
    // INPUTS:
    //     V   Freestream airspeed, m/s
    //     n   Propeller speed, revolutions per second
    double calc_advance_ratio(double V, double n);

    virtual double calc_coeff_thrust(double ar) = 0;
    virtual double calc_coeff_power(double ar) = 0;

    // Calculate thrust of propeller.
    double calc_thrust(double V, double n, double rho);

    // Calculate power of propeller.
    double calc_power(double V, double n, double rho);

    // Calculate torque of propeller.
    double calc_torque(double V, double n, double rho);

    // Calculate efficiency of propeller from advance ratio.
    double calc_efficiency(double ar);

    uav_utils::Wrench_t calc_wrench(double V, double n, double rho);

private:
    // The maximum drag the propeller produces when freewheeling.
    //
    // Used to cap the negative force produced, since the propeller will stall.
    // A flat plate model is used.
    double max_drag(double V);
};

class PropellerStandard : public Propeller
{
public:
    double pitch{0};
    Polynomial *c_thrust, *c_power;

    PropellerStandard(string name) : Propeller(name) {};

    void initialize_parameters() override;
    void update_parameters() override;
    double calc_coeff_thrust(double ar) override;
    double calc_coeff_power(double ar) override;
};

std::unique_ptr<Thruster> buildThruster(ParameterManager propConfig);


} // namespace propulsion
} // namespace last_letter_lib
