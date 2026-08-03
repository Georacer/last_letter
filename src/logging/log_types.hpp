#pragma once

// Backend-aware logging header. It provides two things for the .cpp files that
// self-log (systems, aerodynamics, propulsion, uav_model):
//
//  1. DataTamer TypeDefinition specializations for the last_letter_lib data
//     types we log. Each must live in the SAME namespace as the type it
//     describes, so DataTamer finds it via ADL, and must return a
//     std::string_view type name and enumerate the type's fields via
//     add("name", &member). Everything must bottom out in numeric scalars.
//  2. The definition of logging::LogChannel::register_value (declared in the
//     backend-free logging.hpp). Defining it here — where DataTamer is included
//     — keeps DataTamer out of logging.hpp while letting callers register values
//     through the opaque handle instead of naming DataTamer directly.
//
// Include this header (not the DataTamer headers) in the .cpp files that log.

#include <memory>
#include <string_view>

#include <Eigen/Eigen>

#include "data_tamer/data_tamer.hpp"

#include <last_letter_lib/logging.hpp>
#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

// ---- logging::LogChannel::register_value ------------------------------------
// Out-of-line definition of the handle's registration template. Recovers the
// type-erased DataTamer channel and forwards to its registerValue(), whose T is
// resolved against the TypeDefinition overloads below via ADL.
namespace last_letter_lib
{
namespace logging
{
template <typename T>
void LogChannel::register_value(const std::string &name, const T *value)
{
    std::static_pointer_cast<DataTamer::LogChannel>(impl_)->registerValue(name, value);
}
} // namespace logging
} // namespace last_letter_lib

// ---- Eigen ------------------------------------------------------------------
// Defined in namespace Eigen so ADL finds it for Eigen::Vector3d. Uses the
// accessor form (&v.x()), mirroring DataTamer's own PseudoEigen example.
namespace Eigen
{
template <typename AddField>
inline std::string_view TypeDefinition(Vector3d &v, AddField &add)
{
    add("x", &v.x());
    add("y", &v.y());
    add("z", &v.z());
    return "Vector3d";
}
} // namespace Eigen

// ---- last_letter_lib::math_utils -------------------------------------------
namespace last_letter_lib
{
namespace math_utils
{
template <typename AddField>
inline std::string_view TypeDefinition(UnitQuaternion &q, AddField &add)
{
    add("w", &q.w());
    add("x", &q.x());
    add("y", &q.y());
    add("z", &q.z());
    return "UnitQuaternion";
}
} // namespace math_utils
} // namespace last_letter_lib

// ---- last_letter_lib::uav_utils --------------------------------------------
namespace last_letter_lib
{
namespace uav_utils
{

template <typename AddField>
inline std::string_view TypeDefinition(Wrench_t &w, AddField &add)
{
    add("force", &w.force);
    add("torque", &w.torque);
    return "Wrench_t";
}

template <typename AddField>
inline std::string_view TypeDefinition(WrenchSum_t &w, AddField &add)
{
    add("gravity", &w.wrenchGrav);
    add("aero", &w.wrenchAero);
    add("prop", &w.wrenchProp);
    add("ground", &w.wrenchGround);
    add("external", &w.wrenchExternal);
    return "WrenchSum_t";
}

template <typename AddField>
inline std::string_view TypeDefinition(Airdata &a, AddField &add)
{
    add("airspeed", &a.airspeed);
    add("alpha", &a.alpha);
    add("beta", &a.beta);
    return "Airdata";
}

template <typename AddField>
inline std::string_view TypeDefinition(Pose &p, AddField &add)
{
    add("position", &p.position);
    add("orientation", &p.orientation);
    return "Pose";
}

template <typename AddField>
inline std::string_view TypeDefinition(Twist &t, AddField &add)
{
    add("linear", &t.linear);
    add("angular", &t.angular);
    return "Twist";
}

} // namespace uav_utils
} // namespace last_letter_lib
