#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {

//////////////////////////
// Define Dynamics class
//////////////////////////

void Dynamics::initialize(ParameterManager config)
{
    Parametrized::initialize(config);

    auto p_worldConfig = config.filter("world/");
    auto p_aeroConfig = config.filter("aero/");
    auto p_propConfig = config.filter("prop/");
    auto p_groundConfig = config.filter("ground/");

    // Create and initialize aerodynamic objects array
    nWings = p_aeroConfig.get<int>("nWings");
    aerodynamics.clear();
    for (int i = 0; i < nWings; i++)
    {
        ParameterManager aeroConfig = p_aeroConfig.filter("airfoil" + std::to_string(i + 1) + "/");
        aeroConfig.register_child_mngr(p_worldConfig); // Component reads world/deltaT.
        auto new_aero = aerodynamics::buildAerodynamics(aeroConfig);
        aerodynamics.push_back(new_aero.get());
        add_child(*new_aero);
        components.push_back(std::move(new_aero));
    }

    // Create and initialize motor objects array
    nMotors = p_propConfig.get<int>("nMotors");
    thrusters.clear();
    for (int i = 0; i < nMotors; i++)
    {
        ParameterManager propConfig = p_propConfig.filter("motor" + std::to_string(i + 1) + "/");
        propConfig.register_child_mngr(p_worldConfig);
        auto new_thruster = propulsion::buildThruster(propConfig);
        thrusters.push_back(new_thruster.get());
        add_child(*new_thruster);
        components.push_back(std::move(new_thruster));
    }

    // Create and initialize body objects array. A tree assembled without
    // loadModelConfig can lack the body section, which means no bodies.
    nBodies = config.exists("body/nBodies") ? config.get<int>("body/nBodies") : 0;
    bodies.clear();
    for (int i = 0; i < nBodies; i++)
    {
        ParameterManager bodyConfig = config.filter("body/body" + std::to_string(i + 1) + "/");
        bodyConfig.register_child_mngr(p_worldConfig);
        auto new_body = body::buildBody(bodyConfig);
        bodies.push_back(new_body.get());
        add_child(*new_body);
        components.push_back(std::move(new_body));
    }

    // Create and initialize ground reactions object
    p_groundConfig.register_child_mngr(p_worldConfig); // GroundReaction reads world/deltaT.
    ground_reaction = ground_reaction::buildGroundReaction(p_groundConfig);
    add_child(*ground_reaction);
}

void Dynamics::load_parameters(ParameterManager config)
{
    Parametrized::load_parameters(config);

    for (int i = 0; i < (int)thrusters.size(); i++) {
        thrusters.at(i)->load_parameters(config.filter("prop/motor" + std::to_string(i + 1) + "/"));
    }
    for (int i = 0; i < (int)aerodynamics.size(); i++) {
        aerodynamics.at(i)->load_parameters(config.filter("aero/airfoil" + std::to_string(i + 1) + "/"));
    }
    for (int i = 0; i < (int)bodies.size(); i++) {
        bodies.at(i)->load_parameters(config.filter("body/body" + std::to_string(i + 1) + "/"));
    }
    if (ground_reaction) {
        ground_reaction->load_parameters(config.filter("ground/"));
    }
}

void Dynamics::update_parameters()
{
    for (auto &component : components) {
        component->update_parameters();
    }
    if (ground_reaction) {
        ground_reaction->update_parameters();
    }
}

// Order subsystems to store control input
void Dynamics::setInput(const Input &input)
{
    for (auto &thruster : thrusters) {
        thruster->setInput(input);
    }
    for (auto &airfoil : aerodynamics) {
        airfoil->setInput(input);
    }
    for (auto &body : bodies) {
        body->setInput(input);
    }
    ground_reaction->setInput(input);
}

// Order subsystems to store control input, passed as PWM micorseconds
void Dynamics::setInputPwm(const InputPwm_t &input)
{
    for (auto &thruster : thrusters) {
        thruster->setInputPwm(input);
    }
    for (auto &airfoil : aerodynamics) {
        airfoil->setInputPwm(input);
    }
    for (auto &body : bodies) {
        body->setInputPwm(input);
    }
    ground_reaction->setInputPwm(input);
}

// Update the timestep in all Components and other related members.
void Dynamics::set_dt(double dt_p)
{
    for (auto &component : components) {
        component->set_dt(dt_p);
    }
    if (ground_reaction) {
        ground_reaction->set_dt(dt_p);
    }
}

void Dynamics::update_local_state(SimState_t states, Environment_t environment)
{
    for (auto &component : components) {
        component->update_local_state(states, environment);
    }
}

// Calculate the forces and torques for each Wrench_t source
void Dynamics::calc_model(SimState_t states)
{
    // First pass: calculate the Center of Mass from the component masses. The
    // equations of motion in Kinematics are written about the CoM, so the inertia
    // tensor and wrench moments must be referenced to it (not the body origin).
    double total_mass = 0.0;
    Vector3d first_moment = Vector3d::Zero(); // sum of mass * position
    for (auto &component : components) {
        total_mass += component->inertial.mass;
        first_moment += component->inertial.mass * component->relative_pose.position;
    }
    cog = (total_mass > 0.0) ? Vector3d(first_moment / total_mass) : Vector3d::Zero();

    // Second pass: evaluate wrenches and accumulate the inertia about the CoM.
    WrenchSum_t new_wrench_sum;
    Inertial new_inertial;
    new_inertial.tensor = Eigen::Matrix3d::Zero();
    for (auto &component : components) {
        component->calc_model();
        new_wrench_sum += component->rotate_wrenches();

        auto rotated_inertial = component->rotate_inertia(cog);
        new_inertial.mass += rotated_inertial.mass;
        new_inertial.tensor += rotated_inertial.tensor;
    }

    inertial = new_inertial;
    int res = math_utils::is_pos_def(inertial.tensor);
    if (!(res == 0))
    {
        switch (res)
        {
        case -1:
            throw runtime_error("Matrix of inertia is singular");
            break;
        case -2:
            throw runtime_error("Matrix of inertia is not positive definite");
            break;
        default:
            break;
        }
    }

    // Call ground reactions routines - MUST BE CALLED LAST!!!
    // This is needed for some ground reactions models, which are designed to counter the remaining force sum
    new_wrench_sum.wrenchGround.force = ground_reaction->getForce(states, new_wrench_sum);
    if (!new_wrench_sum.wrenchGround.force.allFinite())
    {
        throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction force vector");
    }

    new_wrench_sum.wrenchGround.torque = ground_reaction->getTorque(states, new_wrench_sum);
    if (!new_wrench_sum.wrenchGround.torque.allFinite())
    {
        throw runtime_error("dynamicsLib.cpp: NaN member in groundReaction torque vector");
    }

    wrench_sum = new_wrench_sum;
}

} // namespace last_letter_lib
