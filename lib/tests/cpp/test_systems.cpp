#include "yaml-cpp/yaml.h"
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <stdexcept>

#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/systems.hpp"

using namespace std;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib::systems;
using namespace last_letter_lib;

class ComponentTest : public ::testing::Test
{
protected:
    void SetUp() override { c.initialize(); }

    Component c{"comp_name"};
};

TEST_F(ComponentTest, TestComponent1)
{
    ASSERT_EQ(c.get_param<double>("inertial/mass"), 0.0);
}

TEST_F(ComponentTest, TestComponent2)
{
    c.set_param("inertial/mass", 1.0);
    ASSERT_EQ(c.get_param<double>("inertial/mass"), 1.0);
}

TEST_F(ComponentTest, TestComponent3)
{
    c.set_param("inertial/mass", 1.0);
    c.update_parameters();
    ASSERT_EQ(c.inertial.mass, 1.0);
}

TEST_F(ComponentTest, TestComponent4)
{
    c.set_param("inertial/tensor/j_xx", 1.0);
    c.set_param("inertial/tensor/j_yy", 1.0);
    c.set_param("inertial/tensor/j_zz", 1.0);
    c.update_parameters();
    ASSERT_EQ(c.inertial.tensor, Eigen::MatrixXd::Identity(3, 3));
}

TEST_F(ComponentTest, TestComponent5)
{
    ParameterManager pm("pm");
    pm.set("inertial/mass", 5.0, false);
    Component c2{"c2"};
    c2.initialize(pm);
    ASSERT_EQ(c2.inertial.mass, 5.0);
}

TEST_F(ComponentTest, TestInitialize)
{
    SimState_t state;
    Environment_t environment;
    c.initialize("pose:\n position:\n  x: 1\ninertial:\n mass: 5.0");
    c.update_local_state(state, environment);
    EXPECT_EQ(c.get_param<double>("pose/position/x"), 1);
    EXPECT_EQ(c.get_param<double>("inertial/mass"), 5);
    EXPECT_EQ(c.local_state.pose.position.x(), 1);
    EXPECT_EQ(c.inertial.mass, 5);
}

class DynamicSystemChild : public DynamicSystem
{
public:
    DynamicSystemChild(stateType x, stateType u) : DynamicSystem(x, u) { };
    DynamicSystemChild() : DynamicSystem() { };
    ~DynamicSystemChild() {};

    stateType dynamics(const stateType x, const stateType u, const double /*t*/) {
        double beta = 0.99; // The time constant is about 1s.
        const double dxdt_0 = (-x[0] + u[0])/beta;
        stateType dxdt = {dxdt_0};
        return dxdt;
    }

    stateType outputs(const stateType x, const stateType /*u*/, const double /*t*/) {
        return x;
    }
};

class DynamicSystemTest : public ::testing::Test
{
public:
    void SetUp() override {
        stateType x_0{0};
        stateType u_0{0};
        system = new DynamicSystemChild(x_0, u_0);
    }

    DynamicSystemChild *system;
};

TEST_F(DynamicSystemTest, TestDynamicSystem)
{
    stateType u={1};
    const double dt = 0.01;
    const double rise_time = 0.35 * 2 * M_PI;
    const int steps = rise_time/dt;
    for (int i=0; i<steps; i++) {
        system->step_dynamics(u, dt);
    }
    auto x = system->x;
    double t = system->t;
    const double x_end = (1 - exp(-rise_time));
    EXPECT_NEAR(system->outputs(x, u, t)[0], x_end, 1e-2);
}

TEST(DynamicSystemTest2, EmptyConstructor)
{
    stateType u={1};
    auto system = DynamicSystemChild();
    stateType x={0};
    system.reset(x, u);
    system.step_dynamics(u, 1e-2);
    EXPECT_GT(system.outputs(system.x, u, system.t)[0], 0);
}

class DynamicSystemEmpty : public DynamicSystem
{
public:
    DynamicSystemEmpty() : DynamicSystem() { };
    ~DynamicSystemEmpty() {};

    stateType dynamics(const stateType /*x*/, const stateType /*u*/, const double /*t*/) {
        stateType dxdt;
        return dxdt;
    }

    stateType outputs(const stateType /*x*/, const stateType u, const double /*t*/) {
        return u;
    }
};

TEST(DynamicSystemTest2, StaticSystem)
{
    auto system = DynamicSystemEmpty();
    stateType u={123};
    system.step_dynamics(u, 1e-2);
    EXPECT_EQ(system.outputs(system.x, u, system.t)[0], 123);
}
