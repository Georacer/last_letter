// Core class definitions
#include <stdexcept>

#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/prog_utils.hpp"

#include "last_letter_lib/logging.hpp"
#include "last_letter_lib/log_types.hpp"

using namespace std;
using namespace last_letter_lib::programming_utils;

namespace last_letter_lib
{

/////////////////////////
// Define UavModel class
//////////////////////////

///////////////////
// Class Constructor
UavModel::UavModel(string name) :
    Parametrized(name),
    kinematics(),
    dynamics()
{
}

void UavModel::initialize(ParameterManager config)
{
    Parametrized::initialize(config);
    // By now own parameters have been initialized, loaded and updated.
    // Proceed to instantiate members.

    init();

    // // Add sensors to the UAV
    // sensors.push_back(std::make_shared<Imu>());
    // sensors.push_back(std::make_shared<Barometer>());
    // sensors.push_back(std::make_shared<AirdataSensor>());
    // sensors.push_back(std::make_shared<Gnss>());
    // sensors.push_back(std::make_shared<MavlinkHilStateQuaternion>());
    // for (auto sensor_ptr : sensors)
    // {
    //     sensor_ptr->init(config);
    // }

    auto kinematics_config = config.filter("kinematics/");
    kinematics_config.register_child_mngr(config.filter("world/")); // Point kinematics to the required world parameters.
    kinematics.initialize(kinematics_config);
    add_child(kinematics);

    auto dynamics_config = config.filter("dynamics/");
    dynamics_config.register_child_mngr(config.filter("world/")); // Point dynamics to the required world parameters.
    dynamics.initialize(dynamics_config);
    add_child(dynamics);

    auto env_params = config.filter("env/");
    env_params.register_child_mngr(params_.filter("world/"));
    environmentModel.initialize(env_params);
    add_child(environmentModel);

    // Initialize input
    setInput(initCtrlInput_);
}

void UavModel::initialize(const std::string yaml_str)
{
    auto config = ParameterManager("aircraft", YAML::Load(yaml_str));

    initialize(config);
}

void UavModel::initialize_parameters()
{
    set_param("world/deltaT", 0.0025, false);

    // Hold these parameters for others to use.
    set_param("world/simRate", 400, false);
    // Simulation time controls
    //  0 : default real-time simulation clock
    //  1 : controls-triggered simulation clock
    //  2 : free-spinning simulation clock
    // 3 : manual step trigger by pause button
    set_param("world/timeControls", 0, false);
    set_param("world/integratorType", 0, false);
    set_param("world/kinematics_engine", 1, false);

    vector<double> position_init = {0, 0, -0.2};
    set_param("init/position", position_init, false); // Initial NED coordinates.
    vector<double> quaternion_init = { 0, 0, 0, 1};
    set_param("init/orientation", quaternion_init, false);
    vector<double> velocity_linear_init = {0, 0, 0};
    set_param("init/velLin", velocity_linear_init, false);
    vector<double> velocity_angular_init = {0, 0, 0};
    set_param("init/velAng", velocity_angular_init, false);
    vector<double> coordinates_init = {0, 0, 0 - position_init[2]}; // Raise the WGS coordinate by the NED altitude.
    set_param("init/coordinates", coordinates_init, false);
    set_param("init/chanReset", -1, false);
    vector<double> ctrl_init = {0, 0, 0, 0};
    set_param("init/ctrlInput", ctrl_init, false);
}

void UavModel::update_parameters()
{
    dt = get_param<double>("world/deltaT");

    vector<double> double_vect;
    double_vect = get_param<vector<double>>("init/position");
    initPosition_ = Vector3d(double_vect[0], double_vect[1], double_vect[2]);
    double_vect = get_param<vector<double>>("init/orientation");
    initOrientation_ = Quaterniond(double_vect[3], double_vect[0], double_vect[1], double_vect[2]);
    double_vect = get_param<vector<double>>("init/velLin");
    initVelLinear_ = Vector3d(double_vect[0], double_vect[1], double_vect[2]);
    double_vect = get_param<vector<double>>("init/velAng");
    initVelAngular_ = Vector3d(double_vect[0], double_vect[1], double_vect[2]);
    double_vect = get_param<vector<double>>("init/coordinates");
    initCoordinates_ = Vector3d(double_vect[0], double_vect[1], double_vect[2] - initPosition_.z()); // Read ground geoid altitude and raise the WGS coordinate by the NED altitude
    initChanReset_ = get_param<int>("init/chanReset");
    initCtrlInput_ = Input(get_param<vector<double>>("init/ctrlInput"));

    // Propagate the update_parameters downwards.
}

// Initialize states
void UavModel::init()
{
    // TODO: All initialization state is not used, passed by Gazebo
    // Read initial NED coordinates
    state.pose.position = initPosition_;

    // Read initial orientation quaternion
    state.pose.orientation = initOrientation_;

    // Read initial velocity
    state.velocity.linear = initVelLinear_;

    // Read initial angular velocity
    state.velocity.angular = initVelAngular_;

    // Initialize rotorspeed vector
    fill(state.rotorspeed.begin(), state.rotorspeed.end(), 0.01);
    // And propulsion model omega
    for (auto &thruster : dynamics.thrusters) {
        thruster->omega = 0.01; // A small non-zero value
    }

    // Initialize WGS coordinates
    state.geoid.latitude = initCoordinates_(0);
    state.geoid.longitude = initCoordinates_(1);
    state.geoid.altitude = initCoordinates_(2);

    chanReset = initChanReset_;
}

///////////////////////////////////////
// Make one step of the plane simulation
void UavModel::step()
{
    // Perform step actions serially

    // Stamp this step's samples with the current sim-time and log the state at
    // the top of the step, so UavModel::state and the component wrenches (which
    // are computed from this same state, mid-step) all share the timestamp.
    log_state();

    // Update the environment and propagate the current state and environment
    // into every component before evaluating their forces and torques.
    environmentModel.calcEnvironment(state);
    dynamics.update_local_state(state, environmentModel.environment);

    // Calculate dynamics
    dynamics.calc_model(state);
    auto inpWrench = dynamics.wrench_sum.sum();

    // Transfer the total moment from the body-frame origin to the Center of
    // Mass, so it is consistent with the CoM-referenced equations of motion
    // and the CoM-referenced inertia tensor. This compensates for moments
    // created by gravity.
    inpWrench.torque -= dynamics.cog.cross(inpWrench.force);

    SimState_t newState;
    newState = kinematics.propagateState(state, dynamics.inertial, inpWrench);

    // Check new state for non-finite values
    Vector3d tempVect;
    Quaterniond tempQuat;
    if (!newState.pose.position.allFinite())
    {
        tempVect = newState.pose.position;
        cout << "New position:\n"
                << tempVect << endl;
        throw runtime_error("uav_model.cpp: NaN member in new position");
    }
    if (!myisfinite(newState.pose.orientation))
    {
        tempQuat = newState.pose.orientation;
        cout << "New orientation:\n"
                << tempQuat.w() << "\n"
                << tempQuat.vec() << endl;
        throw runtime_error("uav_model.cpp: NaN member in new orientation");
    }
    if (!newState.velocity.linear.allFinite())
    {
        tempVect = newState.velocity.linear;
        cout << "New linear velocity:\n"
                << tempVect << endl;
        throw runtime_error("uav_model.cpp: NaN member in new linear velocity");
    }
    if (!newState.velocity.angular.allFinite())
    {
        tempVect = newState.velocity.angular;
        cout << "New angular velocity:\n"
                << tempVect << endl;
        throw runtime_error("uav_model.cpp: NaN member in new angular velocity");
    }

    for (int i = 0; i < (int)dynamics.thrusters.size(); i++)
    {
        newState.rotorspeed[i] = dynamics.thrusters.at(i)->omega;
    }

    state = newState;

    // Advance the simulation clock.
    sim_time_ += dt;
}

/////////////////////////////////////////////////
// Data logging
void UavModel::enable_logging(const std::string &path)
{
    logging::enable(path);
}

void UavModel::disable_logging()
{
    logging::disable();
}

void UavModel::log_state()
{
    if (!logging::is_enabled())
    {
        return;
    }
    logging::set_time(sim_time_);
    // (Re)register our channel for this recording the first time we log into
    // it; enable() clears the registry and bumps the epoch.
    auto channel = logging::get_channel(get_name());
    if (log_epoch_ != logging::epoch())
    {
        channel.register_value("pose", &state.pose);
        channel.register_value("velocity", &state.velocity);
        log_epoch_ = logging::epoch();
    }
    channel.take_snapshot();
}

/////////////////////////////////////////////////
// Pass control inputs to sub-models
void UavModel::setInput(const Input p_input)
{
    input.value = p_input.value;
    dynamics.setInput(input);

    if (chanReset > -1)
    { // If a reset channel is set
        if (input.value[chanReset] > 0.6)
        { // Reset the simulation upon command
            init();
        }
    }
}

/////////////////////////////////////////////////
// convert uS PWM values to control surface inputs
void UavModel::setInputPwm(const InputPwm_t p_input)
{
    PwmInput.value = p_input.value;
    dynamics.setInputPwm(PwmInput);

    if (chanReset > -1)
    { // If a reset channel is set
        if (PwmInput.value[chanReset] > 1600)
        { // Reset the simulation upon PWM command
            init();
        }
    }
}

} // namespace last_letter_lib
