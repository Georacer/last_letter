#define PYBIND11_DETAILED_ERROR_MESSAGES

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>

#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/systems.hpp"
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/body/body.hpp"
#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/body/linear_copter.hpp"
#include "last_letter_lib/control/control_map.hpp"
#include "last_letter_lib/control/evaluate.hpp"
#include "last_letter_lib/control/linearize.hpp"
#include "last_letter_lib/control/trim.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/logging.hpp"

namespace py = pybind11;

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using last_letter_lib::GravitySimple;
using last_letter_lib::GravityClassic;
using last_letter_lib::Environment_t;
using last_letter_lib::EnvironmentModel;
using last_letter_lib::math_utils::EulerAngles;
using last_letter_lib::math_utils::Inertial;
using last_letter_lib::math_utils::UnitQuaternion;
using Vector3_ll = last_letter_lib::math_utils::Vector3;
using last_letter_lib::programming_utils::Parametrized;
using last_letter_lib::systems::Component;
using last_letter_lib::uav_utils::Airdata;
using last_letter_lib::uav_utils::Input;
using last_letter_lib::uav_utils::Pose;
using last_letter_lib::uav_utils::SimState_t;
using last_letter_lib::uav_utils::Wrench_t;
using last_letter_lib::uav_utils::WrenchSum_t;
using last_letter_lib::UavModel;
using namespace last_letter_lib::aerodynamics;
using namespace last_letter_lib::propulsion;

class PyParametrized : public Parametrized
{
public:
    // Inherit the constructor
    using Parametrized::Parametrized;

    // Create trampolines for each virtual function.
    // TODO: Why do we need to bind this method? Is it because we need to always call it after instantiation?
    void initialize_parameters() override
    {
        PYBIND11_OVERRIDE(
            void,                  // Return type
            Parametrized,          // Parent class
            initialize_parameters, // Name of function in C++ (must match Python name)
            // Argument(s)
        );
    }
    void update_parameters() override
    {
        PYBIND11_OVERRIDE_PURE(
            void,              // Return type
            Parametrized,      // Parent class
            update_parameters, // Name of function in C++ (must match Python name)
            // Argument(s)
        );
    }
};

UnitQuaternion unit_quaternion_from_euler_angles(EulerAngles euler) { return UnitQuaternion(euler); }
UnitQuaternion unit_quaternion_from_rotmat(Matrix3d R) { return UnitQuaternion(R); }
EulerAngles euler_angles_from_unit_quaternion(UnitQuaternion q) { return EulerAngles(q); }
EulerAngles euler_angles_from_rotmat(Matrix3d R) { return EulerAngles(R); }
SimState_t simstate_from_array(const VectorXd &v) { return SimState_t(v); }
SimState_t simstate_from_simstate(const SimState_t &s) { return SimState_t(s); }
Inertial inertial_simple(const double mass, const double j_xx, const double j_yy, const double j_zz)
{
    std::vector<double> v = {j_xx, j_yy, j_zz};
    return Inertial(mass, v);
}

PYBIND11_MODULE(cpp_last_letter, m)
{
    m.doc() = "The last_letter_lib python bindings.";

    // Data-logging controls. The data itself is gathered in C++; Python turns
    // recording on and off and decides when a frame is written. log_frame() is
    // what puts a row in the file, and it writes every channel at one
    // timestamp. Aircraft.step() calls it for you.
    m.def("enable_logging", &last_letter_lib::logging::enable, py::arg("path") = "",
          "Start recording simulation data to an MCAP file at `path`. With no "
          "path, a file named after the current date/time is created in the cwd.");
    m.def("disable_logging", &last_letter_lib::logging::disable,
          "Stop recording and close the MCAP file.");
    m.def("log_frame", py::overload_cast<double>(&last_letter_lib::logging::log_frame),
          py::arg("t"),
          "Write one row for every registered channel, stamped with simulation "
          "time `t` in seconds. Does nothing while logging is off.");
    m.def("log_frame", py::overload_cast<>(&last_letter_lib::logging::log_frame),
          "Write one row for every registered channel, stamped with a frame "
          "index counting from 0. Does nothing while logging is off.");

    auto m_math_utils = m.def_submodule("cpp_math_utils", "last_letter_lib math_utils submodule");
    py::class_<Vector3_ll>(m_math_utils, "Vector3")
        .def(py::init<double, double, double>(), py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(py::self - py::self)
        .def(py::self -= py::self)
        .def(py::self * double())
        .def(double() * py::self)
        .def(py::self *= double())
        .def(-py::self)
        .def(py::self == py::self)
        .def_property("x", &Vector3_ll::get_x, &Vector3_ll::set_x)
        .def_property("y", &Vector3_ll::get_y, &Vector3_ll::set_y)
        .def_property("z", &Vector3_ll::get_z, &Vector3_ll::set_z)
        .def_property_readonly("norm", &Vector3_ll::norm)
        .def("to_array", &Vector3_ll::to_array)
        .def(
            "__getitem__",
            [](const Vector3_ll &v, const size_t idx)
            { return v[idx]; })
        .def("__str__", &Vector3_ll::to_str)
        .def("__repr__", &Vector3_ll::repr)
        .def(py::pickle(
            [](const Vector3_ll &v)
            { return v.to_array(); },
            [](Vector3d v)
            {
                return Vector3_ll(v.x(), v.y(), v.z());
            }));
    py::class_<UnitQuaternion>(m_math_utils, "UnitQuaternion")
        .def(py::init<double, double, double, double>(), py::arg("w") = 1, py::arg("x") = 0, py::arg("y") = 0, py::arg("z") = 0)
        .def_property("w", &UnitQuaternion::get_w, &UnitQuaternion::set_w)
        .def_property("x", &UnitQuaternion::get_x, &UnitQuaternion::set_x)
        .def_property("y", &UnitQuaternion::get_y, &UnitQuaternion::set_y)
        .def_property("z", &UnitQuaternion::get_z, &UnitQuaternion::set_z)
        .def_property_readonly("real", &UnitQuaternion::real)
        .def_property_readonly("imag", &UnitQuaternion::imag)
        .def("normalize", &UnitQuaternion::normalize)
        .def("conjugate", &UnitQuaternion::conjugate)
        .def("inverse", &UnitQuaternion::inverse)
        .def("flipped", &UnitQuaternion::flipped)
        .def("to_array", &UnitQuaternion::get_coeffs)
        .def("to_prodmat", &UnitQuaternion::to_prodmat)
        .def("to_euler", &UnitQuaternion::to_euler)
        .def("R_ib", &UnitQuaternion::R_ib)
        .def("R_bi", &UnitQuaternion::R_bi)
        .def("q_dot", &UnitQuaternion::q_dot)
        .def("__repr__", &UnitQuaternion::to_str)
        .def(
            "__eq__", [](const UnitQuaternion &a, UnitQuaternion &b)
            { return a.isApprox(b); },
            py::is_operator())
        .def(py::self * py::self)
        .def(py::self *= py::self)
        .def(
            "__mul__", [](const UnitQuaternion q, const Vector3d v)
            { return q * v; },
            py::is_operator())
        .def(
            "__mul__", [](const UnitQuaternion q, const Vector3_ll v)
            { return q * v; },
            py::is_operator())
        .def_static("from_euler", unit_quaternion_from_euler_angles)
        .def_static("from_rotmat", unit_quaternion_from_rotmat)
        // .def_static("from_two_vectors", &UnitQuaternion::UnitQuaternion<Vector3d, Vector3d, double>);
        .def("is_unit", &UnitQuaternion::is_unit)
        .def(py::pickle(
            [](const UnitQuaternion &q)
            { return q.get_coeffs(); },
            [](Vector4d v)
            {
                return UnitQuaternion(v(0), v(1), v(2), v(3));
            }));
    py::class_<EulerAngles>(m_math_utils, "EulerAngles")
        .def(py::init<double, double, double, bool>(), py::arg("roll") = 0, py::arg("pitch") = 0, py::arg("yaw") = 0, py::arg("in_degrees") = false)
        .def_readwrite("roll", &EulerAngles::roll)
        .def_readwrite("pitch", &EulerAngles::pitch)
        .def_readwrite("yaw", &EulerAngles::yaw)
        .def(py::self == py::self)
        .def("__repr__", &EulerAngles::to_str)
        .def_static("from_quaternion", euler_angles_from_unit_quaternion)
        .def_static("from_rotmat", euler_angles_from_rotmat)
        .def("to_quaternion", &EulerAngles::to_quaternion)
        .def("to_array", [](EulerAngles &self)
             { return Vector3d(self.roll, self.pitch, self.yaw); })
        .def("R_roll", &EulerAngles::R_roll)
        .def("R_pitch", &EulerAngles::R_pitch)
        .def("R_yaw", &EulerAngles::R_yaw)
        .def("R_bi", &EulerAngles::R_bi)
        .def("R_ib", &EulerAngles::R_ib)
        .def("T_eb", &EulerAngles::T_eb)
        .def("T_be", &EulerAngles::T_be);
    py::class_<Inertial>(m_math_utils, "Inertial")
        .def(py::init<double>(), py::arg("mass") = 0)
        .def(py::init<double, std::vector<double>>(), py::arg("mass"), py::arg("tensor"))
        .def_readwrite("mass", &Inertial::mass)
        .def_readwrite("tensor", &Inertial::tensor)
        .def_static("simple", inertial_simple);

    auto m_uav_utils = m.def_submodule("cpp_uav_utils", "last_letter_lib uav_utils submodule");
    py::class_<Pose>(m_uav_utils, "Pose")
        .def(py::init())
        .def("__matmul__", &Pose::operator*)
        // .def_property("position", &Pose::get_position_as_vector3, &Pose::set_position_from_vector3)
        .def_readwrite("position", &Pose::position)
        .def_readwrite("orientation", &Pose::orientation)
        .def_property_readonly("T", &Pose::T);
    py::class_<Wrench_t>(m_uav_utils, "Wrench")
        .def(py::init<Vector3d, Vector3d>(), py::arg("force"), py::arg("torque"))
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def_readwrite("force", &Wrench_t::force)
        .def_readwrite("torque", &Wrench_t::torque)
        .def("to_array", &Wrench_t::to_array);
    py::class_<WrenchSum_t>(m_uav_utils, "WrenchSum")
        .def(py::init())
        .def_readwrite("wrench_grav", &WrenchSum_t::wrenchGrav)
        .def_readwrite("wrench_aero", &WrenchSum_t::wrenchAero)
        .def_readwrite("wrench_prop", &WrenchSum_t::wrenchProp)
        .def_readwrite("wrench_ground", &WrenchSum_t::wrenchGround)
        .def_readwrite("wrench_external", &WrenchSum_t::wrenchExternal);
    py::class_<Airdata>(m_uav_utils, "Airdata")
        .def(py::init<double, double, double>(), py::arg("airspeed") = 0, py::arg("alpha") = 0, py::arg("beta") = 0)
        .def("init_from_velocity", &Airdata::init_from_velocity, py::arg("vel_body"), py::arg("vel_wind") = Vector3d::Zero())
        .def("init_from_state_wind", &Airdata::init_from_state_wind, py::arg("state"), py::arg("wind"))
        .def_property_readonly("S_bw", &Airdata::S_bw)
        .def_property_readonly("S_wb", &Airdata::S_wb)
        .def("to_u", &Airdata::to_velocity)
        .def_readwrite("airspeed", &Airdata::airspeed)
        .def_readwrite("alpha", &Airdata::alpha)
        .def_readwrite("beta", &Airdata::beta);
    py::class_<SimState_t>(m_uav_utils, "UavState")
        .def(py::init())
        .def(py::init<Vector3d, UnitQuaternion, Vector3d, Vector3d, std::vector<double>>(),
             py::arg("position") = Vector3d::Zero(),
             py::arg("orientation") = UnitQuaternion(),
             py::arg("velocity_linear") = Vector3d::Zero(),
             py::arg("velocity_angular") = Vector3d::Zero(),
             py::arg("thrusters_velocity") = std::vector<double>(0))
        .def(py::init<Vector3_ll, UnitQuaternion, Vector3_ll, Vector3_ll, std::vector<double>>(),
             py::arg("position") = Vector3_ll(),
             py::arg("orientation") = UnitQuaternion(),
             py::arg("velocity_linear") = Vector3_ll(),
             py::arg("velocity_angular") = Vector3_ll(),
             py::arg("thrusters_velocity") = std::vector<double>(0))
        .def_static("from_array", simstate_from_array)
        .def_static("from_uavstate", simstate_from_simstate)
        .def_property("position", &SimState_t::get_position, &SimState_t::set_position)
        // Fix discrepancy between "attitude" property and "orientation" initialization argument
        .def_property("attitude", &SimState_t::get_orientation, &SimState_t::set_orientation)
        .def_property("velocity_linear", &SimState_t::get_velocity_linear, &SimState_t::set_velocity_linear)
        .def_property("velocity_angular", &SimState_t::get_velocity_angular, &SimState_t::set_velocity_angular)
        .def_readwrite("thrusters_velocity", &SimState_t::rotorspeed)
        .def("get_euler", [](SimState_t &s)
             { return s.pose.orientation.to_euler(); })
        .def("to_array", &SimState_t::to_array)
        .def("strip_thrusters", &SimState_t::strip_thrusters)
        .def("__deepcopy__", [](const SimState_t &self, py::dict)
             { return SimState_t(self); });
    py::class_<Input>(m_uav_utils, "Inputs")
        .def(py::init())
        .def(py::init<const std::vector<double> &>(), py::arg("values"))
        .def("__len__", [](const Input &i)
             { return i.value.size(); })
        .def("__getitem__", [](const Input &i, size_t n)
             { return i.value.at(n); })
        .def("__setitem__", [](Input &i, size_t n, double v)
             { i.value.at(n) = v; })
        .def("to_array", &Input::to_array)
        .def_static("from_array", &Input::from_array, py::arg("values"));

    auto m_programming_utils = m.def_submodule("cpp_programming_utils", "last_letter_lib programming_utils submodule");
    py::class_<Parametrized, PyParametrized>(m_programming_utils, "Parametrized")
        .def(py::init<string>(), py::arg("name"))
        .def("initialize", [](Parametrized &self) { self.initialize(); })
        .def("initialize", static_cast<void (Parametrized::*)(const std::string)>(&Parametrized::initialize), py::arg("parameters"))
        .def("get_param_double", &Parametrized::get_param<double>, py::arg("param_name"))
        .def("get_param_vector", &Parametrized::get_param<vector<double>>, py::arg("param_name"))
        .def("get_param_string", &Parametrized::get_param<std::string>, py::arg("param_name"))
        .def("set_param", &Parametrized::set_param<double>, py::arg("param_name"), py::arg("param_value"), py::arg("safe") = true)
        .def("set_param", &Parametrized::set_param<std::string>, py::arg("param_name"), py::arg("param_value"), py::arg("safe") = true)
        .def("set_param", &Parametrized::set_param<vector<double>>, py::arg("param_name"), py::arg("param_value"), py::arg("safe") = true);
    m_programming_utils.def(
        "load_model_config",
        [](const std::string &name, const std::string &root)
        { return last_letter_lib::programming_utils::loadModelConfig(name, root).str(); },
        py::arg("name"), py::arg("root") = "",
        "Assemble the recipe `name` from the models directory `root` and return it "
        "as one YAML string. An empty root selects $HOME/last_letter_models.");

    auto m_systems = m.def_submodule("cpp_systems", "last_letter_lib systems submodule");
    py::class_<Component, Parametrized>(m_systems, "Component", py::multiple_inheritance()) // Declaring as multiple_inheritance, because Parametrized is a virtual base of Component.
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &Component::update_parameters)
        .def("update_local_state", &Component::update_local_state)
        .def("calc_model", &Component::calc_model)
        .def_readonly("wrench_sum", &Component::wrench_sum)
        .def_readonly("local_state", &Component::local_state)
        .def_readonly("inertial", &Component::inertial);

    auto m_gravity = m.def_submodule("cpp_gravity", "last_letter_lib gravity submodule");
    // TODO: Attempt to bind pure virtual GravityModel.
    py::class_<GravitySimple>(m_gravity, "GravitySimple")
        .def(py::init())
        .def("calc_gravity", &GravitySimple::calcGravity)
        .def("get_wrench", &GravitySimple::getWrench);
    py::class_<GravityClassic>(m_gravity, "GravityClassic")
        .def(py::init())
        .def("calc_gravity", &GravityClassic::calcGravity)
        .def("get_wrench", &GravitySimple::getWrench);

    auto m_environment = m.def_submodule("cpp_environment", "last_letter_lib environment submodule");
    py::class_<Environment_t>(m_environment, "EnvironmentData")
        .def(py::init())
        .def_readwrite("wind", &Environment_t::wind)
        .def_readwrite("rho", &Environment_t::density)
        .def_readwrite("pressure", &Environment_t::pressure)
        .def_readwrite("temperature", &Environment_t::temperature);
    py::class_<EnvironmentModel, Parametrized>(m_environment, "EnvironmentModel", py::multiple_inheritance()) // Declaring as multiple_inheritance, because Parametrized is a virtual base of Component.
        .def(py::init())
        .def("update_parameters", &EnvironmentModel::update_parameters)
        .def("calc_environment", &EnvironmentModel::calcEnvironment)
        .def_readonly("data", &EnvironmentModel::environment);

    auto m_aerodynamics = m.def_submodule("cpp_aerodynamics", "last_letter_lib gravity aerodynamics");
    m_aerodynamics.def("calc_dynamic_pressure", &calc_dynamic_pressure, "Calculate dynamic pressure");
    m_aerodynamics.def("calc_bank_from_radius", &calc_bank_from_radius, "Calculate the required bank angle to sustain a turn");
    py::class_<Aerodynamics, Component>(m_aerodynamics, "Aerodynamics")
        .def("set_input", &Aerodynamics::setInput)
        .def("calc_model", &Aerodynamics::calc_model);
    py::class_<NoAerodynamics, Aerodynamics>(m_aerodynamics, "NoAerodynamics")
        .def(py::init<string>(), py::arg("name"));
    py::class_<StdLinearAero, Aerodynamics>(m_aerodynamics, "StdLinearAero")
        .def(py::init<string>(), py::arg("name"))
        .def("lift_coeff", &StdLinearAero::liftCoeff)
        .def("drag_coeff", &StdLinearAero::dragCoeff);
    py::class_<SimpleDrag, StdLinearAero>(m_aerodynamics, "SimpleDrag")
        .def(py::init<string>(), py::arg("name"));
    py::class_<BluffBody, Aerodynamics>(m_aerodynamics, "BluffBody")
        .def(py::init<string>(), py::arg("name"));
    py::class_<PolynomialAero, StdLinearAero>(m_aerodynamics, "PolynomialAero")
        .def(py::init<string>(), py::arg("name"));

    auto m_body = m.def_submodule("cpp_body", "last_letter_lib body submodule");
    py::class_<last_letter_lib::body::Body, Component>(m_body, "Body")
        .def(py::init<string>(), py::arg("name"))
        .def("set_input", &last_letter_lib::body::Body::setInput)
        .def("calc_model", &last_letter_lib::body::Body::calc_model);
    py::class_<last_letter_lib::body::LinearCopter, last_letter_lib::body::Body>(m_body, "LinearCopter")
        .def(py::init<string>(), py::arg("name"))
        .def("set_input", &last_letter_lib::body::LinearCopter::setInput)
        .def("calc_model", &last_letter_lib::body::LinearCopter::calc_model)
        .def_readonly("d_rol", &last_letter_lib::body::LinearCopter::d_rol)
        .def_readonly("d_pit", &last_letter_lib::body::LinearCopter::d_pit)
        .def_readonly("d_yaw", &last_letter_lib::body::LinearCopter::d_yaw)
        .def_readonly("d_thr", &last_letter_lib::body::LinearCopter::d_thr)
        .def("set_input_pwm", [](last_letter_lib::body::LinearCopter &self, const std::vector<uint16_t> &pwm)
             {
                 last_letter_lib::uav_utils::InputPwm_t input;
                 std::copy_n(pwm.begin(), std::min(pwm.size(), input.value.size()), input.value.begin());
                 self.setInputPwm(input);
             },
             py::arg("pwm"), "Store the inputs that the inverse mixer recovers from the PWM of each channel.")
        .def_readonly("u_rol", &last_letter_lib::body::LinearCopter::u_rol)
        .def_readonly("u_pit", &last_letter_lib::body::LinearCopter::u_pit)
        .def_readonly("u_yaw", &last_letter_lib::body::LinearCopter::u_yaw)
        .def_readonly("u_thr", &last_letter_lib::body::LinearCopter::u_thr);

    // The mixers hold a shared_ptr, since a CopterControlMap shares its mixer
    // with Python. pybind11 refuses a holder that differs from the holder of
    // the base, so Parametrized is not named as the base here.
    py::class_<last_letter_lib::body::MatrixMixer, std::shared_ptr<last_letter_lib::body::MatrixMixer>>(m_body, "MatrixMixer")
        .def("mixer_type", &last_letter_lib::body::MatrixMixer::mixer_type)
        .def("n_motors", &last_letter_lib::body::MatrixMixer::n_motors)
        .def("channels", &last_letter_lib::body::MatrixMixer::channels)
        .def("mix", &last_letter_lib::body::MatrixMixer::mix, py::arg("rpyt"),
             "Ideal [roll, pitch, yaw, throttle] to the thrust fraction of each motor.")
        .def("invert", &last_letter_lib::body::MatrixMixer::invert, py::arg("thrust"),
             "The least squares inverse of mix().")
        .def("mix_to_pwm", &last_letter_lib::body::MatrixMixer::mix_to_pwm, py::arg("rpyt"),
             "Ideal [roll, pitch, yaw, throttle] to the PWM of each channel in microseconds, unrounded.")
        .def("invert_pwm", &last_letter_lib::body::MatrixMixer::invert_pwm, py::arg("pwm"),
             "Recover [roll, pitch, yaw, throttle] from the PWM of each channel.");
    py::class_<last_letter_lib::body::ArduPilotMixer, last_letter_lib::body::MatrixMixer,
               std::shared_ptr<last_letter_lib::body::ArduPilotMixer>>(m_body, "ArduPilotMixer")
        .def("get_compensation_gain", &last_letter_lib::body::ArduPilotMixer::get_compensation_gain)
        .def("get_air_density_ratio", &last_letter_lib::body::ArduPilotMixer::get_air_density_ratio)
        .def("get_battery_ratio", &last_letter_lib::body::ArduPilotMixer::get_battery_ratio);

    auto m_propulsion = m.def_submodule("cpp_propulsion", "last_letter_lib propulsion submodule");
    py::class_<Propeller, Parametrized>(m_propulsion, "Propeller")
        .def("update_parameters", &Propeller::update_parameters)
        .def("calc_advance_ratio", &Propeller::calc_advance_ratio)
        .def("calc_thrust", &Propeller::calc_thrust)
        .def("calc_power", &Propeller::calc_power)
        .def("calc_torque", &Propeller::calc_torque)
        .def("calc_efficiency", &Propeller::calc_efficiency)
        .def("calc_wrench", &Propeller::calc_wrench)
        .def_readonly("diameter", &Propeller::diameter);
    py::class_<PropellerStandard, Propeller>(m_propulsion, "PropellerStandard")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &PropellerStandard::update_parameters)
        .def("calc_coeff_thrust", &PropellerStandard::calc_coeff_thrust)
        .def("calc_coeff_power", &PropellerStandard::calc_coeff_power);
    py::class_<Thruster, Component>(m_propulsion, "Thruster")
        .def("update_parameters", &Thruster::update_parameters)
        .def("set_input", &Thruster::setInput)
        .def("set_input_pwm", &Thruster::setInputPwm)
        .def("set_throttle", &Thruster::set_throttle, py::arg("throttle"),
             "Set the 0-1 throttle directly, skipping the chanMotor lookup.")
        .def("calc_model", &Thruster::calc_model)
        .def_readonly("omega", &Thruster::omega);
    py::class_<NoEngine, Thruster>(m_propulsion, "NoEngine")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &NoEngine::update_parameters)
        .def("set_input", &NoEngine::setInput)
        .def("set_input_pwm", &NoEngine::setInputPwm)
        .def("calc_model", &NoEngine::calc_model);
    py::class_<ThrusterSimple, Thruster>(m_propulsion, "ThrusterSimple")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &ThrusterSimple::update_parameters)
        .def("set_input", &ThrusterSimple::setInput)
        .def("set_input_pwm", &ThrusterSimple::setInputPwm)
        .def("calc_model", &ThrusterSimple::calc_model);
    py::class_<EngBeard, Thruster>(m_propulsion, "EngBeard")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &EngBeard::update_parameters)
        .def("set_input", &EngBeard::setInput)
        .def("set_input_pwm", &EngBeard::setInputPwm)
        .def("calc_model", &EngBeard::calc_model);
    py::class_<ElectricEng, Thruster>(m_propulsion, "ElectricEng")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &ElectricEng::update_parameters)
        .def("set_input", &ElectricEng::setInput)
        .def("set_input_pwm", &ElectricEng::setInputPwm)
        .def("calc_model", &ElectricEng::calc_model);
    py::class_<ElectricEng2, Thruster>(m_propulsion, "ElectricEng2")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &ElectricEng2::update_parameters)
        .def("set_input", &ElectricEng2::setInput)
        .def("set_input_pwm", &ElectricEng2::setInputPwm)
        .def("calc_model", &ElectricEng2::calc_model);
    py::class_<EngOmegaControl, Thruster>(m_propulsion, "EngOmegaControl")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &EngOmegaControl::update_parameters)
        .def("set_input", &EngOmegaControl::setInput)
        .def("set_input_pwm", &EngOmegaControl::setInputPwm)
        .def("calc_model", &EngOmegaControl::calc_model);
    py::class_<PistonEng, Thruster>(m_propulsion, "PistonEng")
        .def(py::init<string>(), py::arg("name"))
        .def("update_parameters", &PistonEng::update_parameters)
        .def("set_input", &PistonEng::setInput)
        .def("set_input_pwm", &PistonEng::setInputPwm)
        .def("calc_model", &PistonEng::calc_model);

    auto m_simulation = m.def_submodule("cpp_simulation", "last_letter_lib simulation submodule");
    py::class_<UavModel, Parametrized>(m_simulation, "Aircraft")
        .def(py::init<string>(), py::arg("name"))
        .def("initialize", static_cast<void (Parametrized::*)(const std::string)>(&Parametrized::initialize), py::arg("parameters"))
        .def("update_parameters", &UavModel::update_parameters)
        .def("set_input", &UavModel::setInput)
        .def("set_input_pwm", &UavModel::setInputPwm)
        .def("step", &UavModel::step)
        .def("init", &UavModel::init)
        .def("set_state", &UavModel::set_state, py::arg("state"),
             "Replace the rigid body state. The internal states of the components stay as they are.")
        .def("enable_logging", &UavModel::enable_logging, py::arg("path") = "")
        .def("disable_logging", &UavModel::disable_logging)
        .def_property_readonly("time", &UavModel::get_time)
        .def_readonly("state", &UavModel::state);

    auto m_control = m.def_submodule("cpp_control", "last_letter_lib control submodule");
    namespace ctl = last_letter_lib::control;
    py::class_<ctl::SettleOptions>(m_control, "SettleOptions")
        .def(py::init())
        .def_readwrite("tolerance", &ctl::SettleOptions::tolerance)
        .def_readwrite("max_iterations", &ctl::SettleOptions::max_iterations);
    py::class_<ctl::TrimTarget>(m_control, "TrimTarget")
        .def(py::init())
        .def_readwrite("position", &ctl::TrimTarget::position)
        .def_readwrite("velocity_ned", &ctl::TrimTarget::velocity_ned)
        .def_readwrite("heading", &ctl::TrimTarget::heading)
        .def_readwrite("yaw_rate", &ctl::TrimTarget::yaw_rate);
    py::class_<ctl::TrimOptions>(m_control, "TrimOptions")
        .def(py::init())
        .def_readwrite("ftol_abs", &ctl::TrimOptions::ftol_abs)
        .def_readwrite("max_evaluations", &ctl::TrimOptions::max_evaluations)
        .def_readwrite("weights", &ctl::TrimOptions::weights)
        .def_readwrite("initial_controls", &ctl::TrimOptions::initial_controls)
        .def_readwrite("settle", &ctl::TrimOptions::settle);
    py::class_<ctl::TrimPoint>(m_control, "TrimPoint")
        .def(py::init())
        .def_readwrite("state", &ctl::TrimPoint::state)
        .def_readwrite("controls", &ctl::TrimPoint::controls)
        .def_readwrite("input", &ctl::TrimPoint::input)
        .def_readwrite("residual", &ctl::TrimPoint::residual)
        .def_readwrite("evaluations", &ctl::TrimPoint::evaluations)
        .def_readwrite("return_code", &ctl::TrimPoint::return_code)
        .def_readwrite("converged", &ctl::TrimPoint::converged);
    py::class_<ctl::LinearizerOptions>(m_control, "LinearizerOptions")
        .def(py::init())
        .def_readwrite("step_velocity", &ctl::LinearizerOptions::step_velocity)
        .def_readwrite("step_rate", &ctl::LinearizerOptions::step_rate)
        .def_readwrite("step_attitude", &ctl::LinearizerOptions::step_attitude)
        .def_readwrite("step_control", &ctl::LinearizerOptions::step_control)
        .def_readwrite("include_gravity", &ctl::LinearizerOptions::include_gravity)
        .def_readwrite("include_coriolis", &ctl::LinearizerOptions::include_coriolis)
        .def_readwrite("include_gyroscopic", &ctl::LinearizerOptions::include_gyroscopic)
        .def_readwrite("settle", &ctl::LinearizerOptions::settle);
    py::class_<ctl::LinearModel>(m_control, "LinearModel")
        .def(py::init())
        .def_readwrite("A_v", &ctl::LinearModel::A_v)
        .def_readwrite("A_w", &ctl::LinearModel::A_w)
        .def_readwrite("A_att", &ctl::LinearModel::A_att)
        .def_readwrite("B", &ctl::LinearModel::B)
        .def_readwrite("options", &ctl::LinearModel::options)
        .def_readwrite("trim", &ctl::LinearModel::trim);
    py::class_<ctl::LinearCopterCoefficients>(m_control, "LinearCopterCoefficients")
        .def(py::init())
        .def_readwrite("Yv", &ctl::LinearCopterCoefficients::Yv)
        .def_readwrite("Ylat", &ctl::LinearCopterCoefficients::Ylat)
        .def_readwrite("Lv", &ctl::LinearCopterCoefficients::Lv)
        .def_readwrite("Lp", &ctl::LinearCopterCoefficients::Lp)
        .def_readwrite("Llat", &ctl::LinearCopterCoefficients::Llat)
        .def_readwrite("Xu", &ctl::LinearCopterCoefficients::Xu)
        .def_readwrite("Xlon", &ctl::LinearCopterCoefficients::Xlon)
        .def_readwrite("Mu", &ctl::LinearCopterCoefficients::Mu)
        .def_readwrite("Mq", &ctl::LinearCopterCoefficients::Mq)
        .def_readwrite("Mlon", &ctl::LinearCopterCoefficients::Mlon)
        .def_readwrite("Nr", &ctl::LinearCopterCoefficients::Nr)
        .def_readwrite("Nped", &ctl::LinearCopterCoefficients::Nped)
        .def_readwrite("Zw", &ctl::LinearCopterCoefficients::Zw)
        .def_readwrite("Zcoll", &ctl::LinearCopterCoefficients::Zcoll)
        .def_readwrite("u_hover", &ctl::LinearCopterCoefficients::u_hover);
    m_control.def(
        "build_mixer",
        [](const std::string &yaml_str)
        {
            return std::shared_ptr<last_letter_lib::body::MatrixMixer>(last_letter_lib::body::buildMixer(
                last_letter_lib::programming_utils::ParameterManager::from_yaml_string("mixer", yaml_str)));
        },
        py::arg("parameters"),
        "Build the mixer that the mixerType key of a mixer/ YAML block selects.");
    py::class_<ctl::ControlMap>(m_control, "ControlMap")
        .def("size", &ctl::ControlMap::size)
        .def("to_input", &ctl::ControlMap::to_input, py::arg("controls"))
        .def("lower_bounds", &ctl::ControlMap::lower_bounds)
        .def("upper_bounds", &ctl::ControlMap::upper_bounds);
    // The map keeps pointers to the thrusters of the aircraft, so the aircraft
    // must live as long as the map.
    py::class_<ctl::CopterControlMap, ctl::ControlMap>(m_control, "CopterControlMap")
        .def(py::init<std::shared_ptr<last_letter_lib::body::MatrixMixer>, const UavModel &>(),
             py::arg("mixer"), py::arg("aircraft"), py::keep_alive<1, 3>());
    py::class_<ctl::IdentityControlMap, ctl::ControlMap>(m_control, "IdentityControlMap")
        .def(py::init<std::vector<int>, Eigen::VectorXd, Eigen::VectorXd>(),
             py::arg("channels"), py::arg("lower"), py::arg("upper"));
    py::class_<ctl::Trimmer>(m_control, "Trimmer")
        .def(py::init<ctl::TrimOptions>(), py::arg("options") = ctl::TrimOptions())
        .def("trim", &ctl::Trimmer::trim, py::arg("aircraft"), py::arg("control_map"),
             py::arg("target") = ctl::TrimTarget(),
             "Find the trim point and leave the aircraft there, state set and components settled.");
    py::class_<ctl::Linearizer>(m_control, "Linearizer")
        .def(py::init<ctl::LinearizerOptions>(), py::arg("options") = ctl::LinearizerOptions())
        .def("linearize", &ctl::Linearizer::linearize, py::arg("aircraft"), py::arg("control_map"),
             py::arg("trim_point"))
        .def_static("to_linear_copter", &ctl::Linearizer::to_linear_copter, py::arg("model"));
}
