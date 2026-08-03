#include <Eigen/Eigen>
#include <gtest/gtest.h>

#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

#include "test_utils.hpp"

using namespace std;

using namespace last_letter_lib;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib::programming_utils;

TEST(TestEnvironment, TestEnvironment1)
{
    ParameterManager config = load_config_aircraft("skywalker_2013");

    EnvironmentModel environment_obj = EnvironmentModel();
    environment_obj.initialize(config.filter("env"));
    SimState_t states;
    states.pose.position = Eigen::Vector3d(0, 0, -10);
    states.geoid.altitude = -states.pose.position.z();
    states.velocity.linear = Eigen::Vector3d(10, 0, 1);

    environment_obj.calcEnvironment(states);
    auto result = environment_obj.environment;

    EXPECT_NEAR(environment_obj.environment.wind.norm(), 5, 1e-2);
    EXPECT_NEAR(environment_obj.environment.density, 1.225, 1e-2);
    EXPECT_NEAR(environment_obj.environment.temperature, 300, 1);
    EXPECT_NEAR(environment_obj.environment.pressure, 1012, 1);
}
