#include <iostream>
#include <Eigen/Eigen>

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/math_utils.hpp"

using namespace std;

using Eigen::Quaterniond;
using Eigen::Vector3d;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::math_utils::Inertial;
using namespace last_letter_lib::programming_utils;

namespace last_letter_lib {

struct Derivatives_t
{
    Vector3d posDot;
    Vector3d speedDot;
    Vector3d rateDot;
    Quaterniond quatDot;
    Vector3d coordDot;
};

// Kinematics equations related classes

// State integrator interface class
// Propagates the state derivatives onto the ModelPlane states
class Integrator
{
public:
    double dt;
    Integrator(ParameterManager worldConfig);
    virtual ~Integrator();
    virtual SimState_t propagation(SimState_t states, Derivatives_t derivatives) = 0;
};

// Forward Euler integrator class
class ForwardEuler : public Integrator
{
public:
    ForwardEuler(ParameterManager worldConfig);
    SimState_t propagation(SimState_t states, Derivatives_t derivatives);
};

Integrator *buildIntegrator(ParameterManager worldConfig);

// Main generic class
class Kinematics : public Parametrized
{
public:
    Kinematics() : Parametrized("kinematics") {};
    ~Kinematics();
    void initialize(ParameterManager config);
    void initialize_parameters();
    void update_parameters();

    // Change the integration timestep.
    void set_dt(double dt_p);

    double dt;
    Derivatives_t stateDot;
    void calcDerivatives(SimState_t states, Inertial inertia, Wrench_t inpWrench);	  // Do not use this method (private)
    SimState_t propagateState(SimState_t states, Inertial inertia, Wrench_t inpWrench); // Use this method to calculate state integral
    Integrator *integrator{nullptr};
};

} // namespace last_letter_lib
