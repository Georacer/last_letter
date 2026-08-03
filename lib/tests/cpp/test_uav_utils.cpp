#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include <last_letter_lib/uav_utils.hpp>

using namespace last_letter_lib::uav_utils;

using Eigen::Quaterniond;
using Eigen::Vector3d;

TEST(testRotations, testNedTf)
{
    // Rotate a vector from NED frame to ENU frame
    // i.e. how vector_ned is seen from ENU
    Vector3d vector_ned{1, 2, 3};
    Vector3d expected_vector_enu{2, 1, -3};
    Vector3d vector_enu = q_ned_enu * vector_ned;
    EXPECT_TRUE(vector_enu.isApprox(expected_vector_enu));

    // Rotate the ENU vector back to NED and compare with the original
    Vector3d new_vector_ned = q_enu_ned * vector_enu;
    EXPECT_TRUE(new_vector_ned.isApprox(vector_ned));
}

TEST(testRotations, testBodyTf)
{
    // Rotate a vector from FLU frame to FRD frame
    Vector3d vector_frd{1, 2, 3};
    Vector3d expected_vector_flu{1, -2, -3};
    Vector3d vector_flu = q_ba_bg * vector_frd;
    EXPECT_TRUE(vector_flu.isApprox(expected_vector_flu));

    // Rotate the FLU vector back to FRD and compare with the original
    Vector3d new_vector_frd = q_bg_ba * vector_flu;
    EXPECT_TRUE(new_vector_frd.isApprox(vector_frd));
}
