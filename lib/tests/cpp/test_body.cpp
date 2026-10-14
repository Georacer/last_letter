#include <gtest/gtest.h>

#include "Eigen/Eigen"

#include "last_letter_lib/body/body.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;
using namespace last_letter_lib::body;

static const string GRAVITY_ONLY_BODY = R"(
name: plain_body
bodyType: 0
inertial:
  mass: 2.0
  tensor:
    j_xx: 0.1
    j_yy: 0.1
    j_zz: 0.2
)";

// A body of type 0 outputs its gravity wrench and nothing else.
TEST(TestBody, TestGravityOnlyBody)
{
    auto body = buildBody(ParameterManager::from_yaml_string("body1", GRAVITY_ONLY_BODY));

    SimState_t state;
    Environment_t environment;
    body->update_local_state(state, environment);
    body->calc_model();

    const double weight = 2.0 * 9.81; // GravitySimple, the default model.
    EXPECT_NEAR(body->wrench_sum.wrenchGrav.force.z(), weight, 1e-9);
    EXPECT_NEAR(body->wrench_sum.wrenchGrav.force.head<2>().norm(), 0.0, 1e-9);
    EXPECT_TRUE(body->wrench_sum.wrenchGrav.torque.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchAero.force.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchAero.torque.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchProp.force.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchProp.torque.isZero());
}

// A recipe without body.yaml gets an empty body section.
TEST(TestBody, TestNoBodyFile)
{
    auto config = load_config_aircraft("skywalker_2013");
    EXPECT_EQ(config.get<int>("dynamics/body/nBodies"), 0);
}
