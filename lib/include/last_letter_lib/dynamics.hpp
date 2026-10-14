#include <Eigen/Eigen>

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/body/body.hpp"
#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/ground_reaction/ground_reaction.hpp"
#include "last_letter_lib/environment.hpp"

using namespace std;

using Eigen::Quaterniond;
using Eigen::Vector3d;

// Dynamic model aggregator class

namespace last_letter_lib
{

// Container class
class Dynamics : public Parametrized
{
public:
    Dynamics() : Parametrized("dynamics") {};
    void initialize(ParameterManager config) override;
    void update_parameters() override;
    void load_parameters(ParameterManager config) override;

    void readParametersAerodynamics(ParameterManager config);
    void readParametersProp(ParameterManager config);
    void readParametersGround(ParameterManager config);
    void readParametersWorld(ParameterManager config);
    void setInput(const Input &input);															   // store and convert new input values
    void setInputPwm(const InputPwm_t &input);														   // store and convert new PWM input values
    void set_dt(double dt_p); // Propagate a new timestep to every component and the ground reaction model.
    void update_local_state(SimState_t states, Environment_t environment);
    void calc_model(SimState_t states); // Calculate the forces and torques for each component.

    WrenchSum_t wrench_sum;
    Inertial inertial; // Lumped inertial characteristics, referenced to the CG.
    Vector3d cog{Vector3d::Zero()}; // Center of gravity w.r.t. the body-frame origin.
    vector<unique_ptr<Component>> components;
    int nWings; // number of airfoils mounted on the aircraft
    vector<aerodynamics::Aerodynamics*> aerodynamics;
    int nMotors; // number of motors mounted on the aircraft
    vector<propulsion::Thruster*> thrusters;
    int nBodies{0}; // number of bodies, components that are neither wings nor motors
    vector<body::Body*> bodies;
    std::unique_ptr<ground_reaction::GroundReaction> ground_reaction;
};

} // namespace last_letter_lib
