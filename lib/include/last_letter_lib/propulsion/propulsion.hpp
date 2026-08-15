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
    double inputMotor;  // control input (0-1)
    int chanMotor;
    double omega;       // motor angular speed in rad/s
    double rotationDir; // motor direction of rotation
    double theta;       // propeller angle in rads
    double normalWind;  // scalar wind normal to propeller disc
    double thrustMax, thrustMin;
    double torqueMax, torqueMin;
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
    // below funnel through this.
    void set_throttle(double throttle) { inputMotor = throttle; }
    void setInput(const Input &input);                                                              // store control input
    void setInputPwm(const InputPwm_t &input);                                                      // store PWM control input
    virtual stateType dynamics(const stateType x, const stateType u, const double t) override;
    virtual stateType outputs(const stateType x, const stateType u, const double t) override;
    void calc_model_impl() override;          // engine physics step, container for the generic class
    virtual void pre_propagation(SimState_t states, Environment_t environment) = 0; // Build the input u of the dynamic system.
    virtual void post_propagation() override = 0; // Distribute the propagated state into other local variables.
    void rotateProp();                                                                              // Update the propeller angle
    virtual void calc_wrench(SimState_t states, Environment_t environment) = 0;     // Calculate Forces
    double torque_sign() {return rotationDir;}

protected:
    void register_log_channels() override; // adds omega/theta/input to base wrench_sum
    bool initialized{false};
};

class NoEngine : public Thruster
{
public:
	NoEngine(string name);
	~NoEngine() {};

    void pre_propagation(SimState_t, Environment_t) override {};
    void post_propagation() override {};
	void calc_wrench(SimState_t, Environment_t) override {};
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
    double propDiam, engInertia, rho;
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

        set_param<double>("propDiam", 0.4064, false);
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
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {-0.00405, 0.00289};
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
    double propDiam, engInertia, rho;
    double Kv, Rm, I0;
    // Battery specification
    int Cells; // Number of LiPo cells
    double Rs; // Battery internal resistance
    double momentumDragCoeff;
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

        set_param<double>("propDiam", 0.4064, false);
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

        set_param<double>("propThrustMultiplier", 1.1, false);
        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {0.00289, -0.00405};
        set_param<vector<double>>("propPowerPoly/coeffs", propPowerPolyCoeffs, false);

        set_param<double>("momentumDragCoeff", 0.0047, false);
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
	double propDiam, engInertia, rho;

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
        set_param<double>("propDiam", 0.5588, false);
        set_param<double>("engInertia", 0.01, false);
        std::vector<double> radpslimits = {80, 630};
        set_param<vector<double>>("RadPSLimits", radpslimits, false);

        // TODO: Replace this with a YAML string?
        set_param<double>("engPowerPoly/polyType", 0, false);
        set_param<double>("engPowerPoly/polyNo", 2, false);
        std::vector<double> engPowerPolyCoeffs = {0, 0.5, -0.005};
        set_param<vector<double>>("engPowerPoly/coeffs", engPowerPolyCoeffs, false);

        set_param<double>("engCoeffPoly/polyType", 0, false);
        set_param<double>("engCoeffPoly/polyNo", 2, false);
        std::vector<double> engCoeffPolyCoeffs = {0, 0.03, -0.0003};
        set_param<vector<double>>("engCoeffPoly/coeffs", engCoeffPolyCoeffs, false);

        set_param<double>("propPowerPoly/polyType", 0, false);
        set_param<double>("propPowerPoly/polyNo", 1, false);
        std::vector<double> propPowerPolyCoeffs = {-0.0405, 0.0289};
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
