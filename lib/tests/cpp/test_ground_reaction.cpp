#include <iostream>

#include "yaml-cpp/yaml.h"
#include <gtest/gtest.h>

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;
using namespace last_letter_lib::ground_reaction;

class GroundReactionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        auto config = load_config_aircraft("skywalker_2013");
        ground_reaction = buildGroundReaction(config.filter("dynamics/ground/"));
    }

    std::unique_ptr<GroundReaction> ground_reaction;
};

TEST_F(GroundReactionTest, TestAboveGround)
{

    SimState_t states;
    states.velocity.linear = Eigen::Vector3d(0,0,0);
    states.pose.position = Eigen::Vector3d(0,0,-1); // Set the contact points above ground, in the air.
    states.geoid.altitude = states.pose.position.z();

    WrenchSum_t wrenchSum;

    Vector3d groundForce = ground_reaction->getForce(states, wrenchSum);
    Vector3d groundTorque = ground_reaction->getTorque(states, wrenchSum);

    ASSERT_EQ(groundForce.x(), 0);
    ASSERT_EQ(groundForce.y(), 0);
    ASSERT_EQ(groundForce.z(), 0);
    ASSERT_EQ(groundTorque.x(), 0);
    ASSERT_EQ(groundTorque.y(), 0);
    ASSERT_EQ(groundTorque.z(), 0);
}

TEST_F(GroundReactionTest, TestImmobileContact)
{

    SimState_t states;
    states.velocity.linear = Eigen::Vector3d(0,0,0);
    states.pose.position = Eigen::Vector3d(0,0,0.1); // Set the contact points above ground, in the air.
    states.geoid.altitude = states.pose.position.z();

    WrenchSum_t wrenchSum;

    Vector3d groundForce = ground_reaction->getForce(states, wrenchSum);

    ASSERT_EQ(groundForce.x(), 0);
    ASSERT_EQ(groundForce.y(), 0);
    ASSERT_LT(groundForce.z(), 0);
}
