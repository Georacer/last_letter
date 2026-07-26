#include "yaml-cpp/yaml.h"
#include <gtest/gtest.h>

#include "last_letter_lib/prog_utils.hpp"
#include "test_utils.hpp"

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib;

YAML::Node load_config_simple()
{

    std::string full_filename = "tests/cpp/test_params.yaml";
    return YAML::LoadFile(full_filename);
}

ParameterManager load_config_aircraft(std::string uav_name, bool randomize)
{
    string paramDir = "tests/cpp/test_parameters/";
    string aircraftDir = "aircraft/";
    string prop_filename = "propulsion.yaml";
    string aero_filename = "aerodynamics.yaml";
    string ground_filename = "ground.yaml";
    string init_filename = "init.yaml";
    string world_filename = "world.yaml";
    string environment_filename = "environment.yaml";
    string randomizer_filename = "randomizer.yaml";

    string fullWorldFilename = paramDir + world_filename;
    string fullEnvironmentFilename = paramDir + environment_filename;
    string fullPropFilename = paramDir + aircraftDir + uav_name + "/" + prop_filename;
    string fullAeroFilename = paramDir + aircraftDir +uav_name + "/" + aero_filename;
    string fullGroundFilename = paramDir + aircraftDir +uav_name + "/" + ground_filename;
    string fullInitFilename = paramDir + aircraftDir +uav_name + "/" + init_filename;

    ParameterManager configs("aircraft");
    ParameterManager world("world", YAML::LoadFile(fullWorldFilename));
    configs.register_child_mngr(world);
    ParameterManager env("env", YAML::LoadFile(fullEnvironmentFilename));
    configs.register_child_mngr(env);

    ParameterManager dynamics_config("dynamics");
    ParameterManager prop("prop", YAML::LoadFile(fullPropFilename));
    dynamics_config.register_child_mngr(prop);
    ParameterManager aero("aero", YAML::LoadFile(fullAeroFilename));
    dynamics_config.register_child_mngr(aero);
    ParameterManager ground("ground", YAML::LoadFile(fullGroundFilename));
    dynamics_config.register_child_mngr(ground);
    configs.register_child_mngr(dynamics_config);

    ParameterManager kinematics_config("kinematics");
    configs.register_child_mngr(kinematics_config);

    ParameterManager init("init", YAML::LoadFile(fullInitFilename));
    configs.register_child_mngr(init);

    // if (randomize)
    // {
    //     string fullRandomizerFilename = paramDir + uav_name + "/" + randomizer_filename;
    //     YAML::Node randomizerConfig = YAML::LoadFile(fullRandomizerFilename);
    //     configs = randomizeConfigsStruct(configs, randomizerConfig);
    // }

    return configs;
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
