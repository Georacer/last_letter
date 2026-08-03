#include "yaml-cpp/yaml.h"
#include <gtest/gtest.h>
#include <string>
#include <stdexcept>

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace std;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib;

TEST(ProgUtilsTest, TestYaml1)
{

    std::string full_filename = test_data_path("test_params.yaml");
    YAML::Node config = YAML::LoadFile(full_filename);
    EXPECT_EQ(config["world"]["simRate"].as<int>(), 500);

    // Test basic parameter access
    std::string timeControls = config["world"]["timeControls"].as<std::string>();
    std::string integratorTypeStr =
        config["world"]["integratorType"].as<std::string>();
    EXPECT_EQ(integratorTypeStr, "0");

    config["lastLogin"] = 5.0;

    // // Test alternate accessor
    // int integratorType;
    // getParameter(config, "world/integratorType", integratorType);
    // EXPECT_EQ(integratorType, 0);

    // std::vector<int> intVector;
    // getParameterList(config, "anIntSequence", intVector);
    // string vectorString = vectorToString(intVector);
    // EXPECT_EQ(vectorString, "0 1 2 ");

    // std::vector<double> doubleVector;
    // getParameterList(config, "aDoubleSequence", doubleVector);
    // vectorString = vectorToString2(doubleVector);
    // EXPECT_EQ(vectorString, "0, 1.1, 2.2, ");
}

TEST(ProgUtilsTest, TestYaml2)
{
    YAML::Node n = YAML::Node();
    ASSERT_TRUE(n.IsNull());
}

TEST(ProgUtilsTest, GeneralTest)
{

    // Read parameter files
    ParameterManager configs = load_config_aircraft("skywalker_2013");

    // Test filter config
    // YAML::Node newConfig = filterConfig(configs.prop, "motor1/");
    // EXPECT_EQ(newConfig["motorType"].as<int>(), 1);

    // Test parameter randomization
    // YAML::Node testConfig = load_config_simple();
    // std::vector<string> paramNames;
    // double doubleValue;
    // getParameter(testConfig, "scalarParameter", doubleValue);
    // EXPECT_EQ(doubleValue, 1);
    // paramNames.push_back("scalarParameter");
    // YAML::Node randomizedConfig = randomizeConfig(testConfig, paramNames, 0.1);
    // getParameter(randomizedConfig, "scalarParameter", doubleValue);
    // EXPECT_NE(doubleValue, 1);

    // Test config struct randomizing
    // ConfigsStruct_t newConfigStruct = load_config_aircraft("skywalker_2013", true);
    // EXPECT_NE(newConfigStruct.aero["airfoil1/c_l_pn"].as<double>(), -1);

    // Test parameter loading from an aircraft models folder. The repository test
    // parameters are used rather than the default ($HOME/last_letter_models).
    ParameterManager loadedConfig = loadModelConfig("skywalker_2013", test_data_path("test_parameters/"));
}

class A
{
public:
    A(string name) : parameters(name)
    {
    }
    ParameterManager parameters;
};

class B
{
public:
    B(string name) : parameters(name)
    {
    }
    void setChild(shared_ptr<A> child_p)
    {
        child_ptr = child_p;
        parameters.register_child_mngr(child_ptr->parameters);
    }
    ParameterManager parameters;
    shared_ptr<A> child_ptr;
};

class ParameterManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        child = make_shared<A>("child");
        child->parameters.set("param_1", 1, false);
        child->parameters.set("param_2/param_3", 3, false);

        parent = make_shared<B>("parent");
        parent->setChild(child);
        parent->parameters.set("param_4", 4, false);
        parent->parameters.set("param_5/param_6", 6, false);
    }

    shared_ptr<A> child;
    shared_ptr<B> parent;
};

TEST_F(ParameterManagerTest, TestAccessParent1)
{
    int r = parent->parameters.get<int>("param_4");
    ASSERT_EQ(r, 4);
}

TEST_F(ParameterManagerTest, TestAccessParent2)
{
    int r = parent->parameters.get<int>("param_5/param_6");
    ASSERT_EQ(r, 6);
}

TEST_F(ParameterManagerTest, TestAccessChild1)
{
    int r = child->parameters.get<int>("param_1");
    ASSERT_EQ(r, 1);
}

TEST_F(ParameterManagerTest, TestAccessChild2)
{
    int r = child->parameters.get<int>("param_2/param_3");
    ASSERT_EQ(r, 3);
}

TEST_F(ParameterManagerTest, TestAccessChild3)
{
    int r = parent->parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 3);
}

TEST_F(ParameterManagerTest, TestExist1)
{
    ASSERT_EQ(child->parameters.exists("param_1"), true);
}

TEST_F(ParameterManagerTest, TestExist2)
{
    ASSERT_EQ(child->parameters.exists("not_existing"), false);
}

TEST_F(ParameterManagerTest, TestInvalidGet)
{
    ASSERT_THROW(child->parameters.get<int>("not_existing"), std::invalid_argument);
}

TEST_F(ParameterManagerTest, TestFilter)
{
    ParameterManager node = parent->parameters.filter("child");
    ASSERT_EQ(node.get<int>("param_1"), 1);
}

TEST_F(ParameterManagerTest, TestKeys)
{
    vector<string> v = child->parameters.keys();
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v[0], "name");
    EXPECT_EQ(v[1], "param_1");
    EXPECT_EQ(v[2], "param_2/param_3");
}

TEST_F(ParameterManagerTest, TestParentWriteChildRead)
{
    parent->parameters.set("child/param_2/param_3", 33, true);
    int r = child->parameters.get<int>("param_2/param_3");
    ASSERT_EQ(r, 33);
}

// Ensure a common child can be manipulated at runtime.
TEST_F(ParameterManagerTest, TestSharedChildNodeAliasing)
{
    auto parent2 = B("parent2");
    parent2.setChild(child);

    int r = parent2.parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 3);

    child->parameters.set("param_2/param_3", 4);
    r = parent2.parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 4);
    r = parent->parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 4);

    parent2.parameters.set("child/param_2/param_3", 5);
    r = parent->parameters.get<int>("child/param_2/param_3");
    ASSERT_EQ(r, 5);
}

TEST(ParameterManagerTest2, TestLoadFile)
{
    auto pm = ParameterManager("pm");
    pm.set("lastLogin", 0, false);
    pm.set("world/simRate", 100, false);
    pm.set("cantfindme", 13, false);
    pm.load_file(test_data_path("test_params.yaml"));

    EXPECT_EQ(pm.get<double>("lastLogin"), 5);
    EXPECT_EQ(pm.get<double>("world/simRate"), 500);
    EXPECT_EQ(pm.get<double>("cantfindme"), 13);
}

TEST(ParameterManagerTest2, TestLoadStream)
{
    auto pm = ParameterManager("pm");
    pm.set("lastLogin", 0, false);
    pm.set("world/simRate", 100, false);
    pm.set("cantfindme", 13, false);
    // std::stringstream ss;
    std::string ss;
    // ss << "lastLogin: 5\nworld:\n simRate: 500\n";
    ss = "lastLogin: 5\nworld:\n simRate: 500\n";
    pm.load_stream(ss);

    EXPECT_EQ(pm.get<double>("lastLogin"), 5);
    EXPECT_EQ(pm.get<double>("world/simRate"), 500);
    EXPECT_EQ(pm.get<double>("cantfindme"), 13);
}

class ConcreteParametrizedChild : public Parametrized
{
public:
    ConcreteParametrizedChild(string name) : Parametrized(name)
    {
        initialize_parameters();
        update_parameters();
    };
    void initialize_parameters()
    {
        set_param("param_1", 1.0, false);
        set_param("param_2", false, false);
    };
    void update_parameters()
    {
        param_1 = get_param<double>("param_1");
        param_2 = get_param<bool>("param_2");
    }

    double param_1;
    bool param_2;
};

class ConcreteParametrizedParent : public Parametrized
{
public:
    ConcreteParametrizedParent(string name) : Parametrized(name)
    {
        initialize_parameters();
        update_parameters();
    };
    void initialize_parameters()
    {
        set_param("param_3", 3.0, false);
        set_param("param_4", true, false);
    };
    void update_parameters()
    {
        param_3 = get_param<double>("param_3");
        param_4 = get_param<bool>("param_4");
    }

    double param_3;
    bool param_4;
};

class ParametrizedTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        child = make_shared<ConcreteParametrizedChild>("child");
        parent = make_shared<ConcreteParametrizedParent>("parent");
        parent->add_child(*child);
    }

    shared_ptr<ConcreteParametrizedChild> child;
    shared_ptr<ConcreteParametrizedParent> parent;
};

TEST_F(ParametrizedTest, TestAccessParent1)
{
    EXPECT_EQ(parent->param_3, 3);
    EXPECT_EQ(parent->param_4, true);
}

TEST_F(ParametrizedTest, TestAccessChild1)
{
    EXPECT_EQ(child->param_1, 1);
    EXPECT_EQ(child->param_2, false);
}

TEST_F(ParametrizedTest, TestAccessChild2)
{
    double r = parent->get_param<double>("child/param_1");
    EXPECT_EQ(r, 1);
}

TEST_F(ParametrizedTest, TestSetChild1)
{
    parent->set_param("child/param_1", 10.0);
    double r = parent->get_param<double>("child/param_1");
    EXPECT_EQ(r, 10);
}
