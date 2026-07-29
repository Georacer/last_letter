#include "last_letter_lib/prog_utils.hpp"
#include "test_utils.hpp"

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib;

// Build the parameter tree of a test aircraft.
ParameterManager load_config_aircraft(std::string uav_name)
{
    return loadModelConfig(uav_name, "tests/cpp/test_parameters/");
}

SimState_t build_aircraft_state_from_config(ParameterManager config)
{
    SimState_t state;
    std::vector<double> doubleVect;
    doubleVect = config.filter("init").get<vector<double>>("velLin");
    state.velocity.linear = Eigen::Vector3d(doubleVect.data());
    doubleVect.clear();
    doubleVect = config.filter("init").get<vector<double>>("velAng");
    state.velocity.angular = Eigen::Vector3d(doubleVect.data());
    doubleVect.clear();
    doubleVect = config.filter("init").get<vector<double>>("position");
    state.pose.position = Eigen::Vector3d(doubleVect.data());
    state.geoid.altitude = -state.pose.position.z();
    // Read initial orientation quaternion
    doubleVect.clear();
    doubleVect = config.filter("init").get<vector<double>>("orientation");
    state.pose.orientation = Quaterniond(doubleVect[3], doubleVect[0], doubleVect[1], doubleVect[2]);

    return state;
}
