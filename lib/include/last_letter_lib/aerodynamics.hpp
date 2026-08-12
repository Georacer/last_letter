/////////////////////////////////////////////
// Aerodynamics class related declarations //
/////////////////////////////////////////////

#pragma once

#include <memory>

#include <Eigen/Eigen>

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/systems.hpp"

using Eigen::Vector3d;

using namespace last_letter_lib;
using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::programming_utils::buildPolynomial;
using last_letter_lib::programming_utils::ParameterManager;

namespace last_letter_lib {
namespace aerodynamics {

double calc_dynamic_pressure(double rho, double v_a);
double calc_bank_from_radius(double R, double v, double gamma, double g); // Calculate the required bank angle to sustain a turn.

//////////////////////////////////////////////
// Aerodynamics interface class declaration //
//////////////////////////////////////////////
class Aerodynamics : public systems::Component
{
public:
    ////////////
    // Variables
    Airdata airdata;
    double p, q, r;                            // rotated angular rates
    double deltaa_max, deltae_max, deltar_max; // Control inputs and maximum surface deflections
    double inputAileron{0}, inputElevator{0}, inputRudder{0};
    int chanAileron, chanElevator, chanRudder;

    ////////////
    // Functions
public:
    Aerodynamics(string name) : Component(name) {};
    virtual ~Aerodynamics() {};
    void initialize_parameters() override;
    void update_parameters() override;
    void setInput(Input input);
    void setInputPwm(InputPwm_t input);
    void calc_model_impl() override; // perform one step in the aerodynamics

protected:
    void register_log_channels() override; // adds airdata to the base wrench_sum
};

///////////////////////////////////////////
// No aerodynamics, create a dummy class //
///////////////////////////////////////////
class NoAerodynamics : public Aerodynamics
{
public:
    NoAerodynamics(string name) : Aerodynamics(name) {};
    ~NoAerodynamics() {};
};

////////////////////////////////////////////////////////
// Typical aerodynamics, linear to their coefficients //
////////////////////////////////////////////////////////
class StdLinearAero : public Aerodynamics
{
public:
    StdLinearAero(string name) : Aerodynamics(name) {};
    ~StdLinearAero() {};
    void initialize_parameters() override;
    void update_parameters() override;
    void calc_model_impl() override;
    // Calculate lift coefficient from alpha
    virtual double liftCoeff(double);
    // Calculate drag coefficient from alpha and beta
    double dragCoeff(double, double);

    double rho, g;
    double c_lift_q, c_lift_deltae, c_drag_q, c_drag_deltae;
    double c, b, s;
    double c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
    double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
    double c_m_0, c_m_a, c_m_q, c_m_deltae;
    double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
    double M, alpha0, c_lift_0, c_lift_a0;
    double c_drag_p, oswald, AR;
private:
    virtual double _dragCoeff(double, double);
};

/////////////////////////////////////////////////////////////////////
// Same as StdLinearAero but with linear-in-the-parameters drag model
/////////////////////////////////////////////////////////////////////
class SimpleDrag : public StdLinearAero
{
public:
    SimpleDrag(string name) : StdLinearAero(name) {};
    ~SimpleDrag() {};
    void initialize_parameters() override;
    void update_parameters() override;

    double c_drag_a;
    // Calculate drag coefficient from alpha
private:
    virtual double _dragCoeff(double, double) override;
};

////////////////////////////////////////////////////////
// Extension for fine specification of the drag polar //
////////////////////////////////////////////////////////
class PolynomialAero : public StdLinearAero
{
public:
    PolynomialAero(string name) : StdLinearAero(name) {};
    ~PolynomialAero() {};
    void initialize_parameters() override;
    void update_parameters() override;

    math_utils::Polynomial *liftCoeffPoly;
    math_utils::Polynomial *dragCoeffPoly;

    double liftCoeff(double alpha) override;
private:
    virtual double _dragCoeff(double, double) override;
};

std::unique_ptr<Aerodynamics> buildAerodynamics(ParameterManager config);

} // namespace aerodynamics
} // namespace last_letter_lib
