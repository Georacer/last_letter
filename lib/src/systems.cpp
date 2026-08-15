#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include <last_letter_lib/systems.hpp>

#include "last_letter_lib/logging.hpp"
#include "logging/log_types.hpp"

namespace last_letter_lib {
namespace systems {

void Component::update_parameters()
{
    relative_pose.position.x() = get_param<double>("pose/position/x");
    relative_pose.position.y() = get_param<double>("pose/position/y");
    relative_pose.position.z() = get_param<double>("pose/position/z");
    relative_pose.orientation.w() = get_param<double>("pose/orientation/w");
    relative_pose.orientation.x() = get_param<double>("pose/orientation/x");
    relative_pose.orientation.y() = get_param<double>("pose/orientation/y");
    relative_pose.orientation.z() = get_param<double>("pose/orientation/z");
    inertial.mass = get_param<double>("inertial/mass");
    inertial.tensor(0, 0) = get_param<double>("inertial/tensor/j_xx");
    inertial.tensor(1, 1) = get_param<double>("inertial/tensor/j_yy");
    inertial.tensor(2, 2) = get_param<double>("inertial/tensor/j_zz");
    inertial.tensor(0, 2) = inertial.tensor(2, 0) = -get_param<double>("inertial/tensor/j_xz");
    dt = get_param<double>("world/deltaT");

    if (gravity == nullptr) {
        gravity = buildGravity(params_.filter("world/gravity/"));
    }
}

void Component::set_dt(double dt_p)
{
    dt = dt_p;
    set_param("world/deltaT", dt_p);
}

void Component::update_local_state(const SimState_t body_state, const Environment_t environment)
{
    // Calculate the local state.
    const UnitQuaternion q_eb = body_state.pose.orientation;
    const UnitQuaternion q_bc = relative_pose.orientation;
    local_state.pose.position = body_state.pose.position + q_eb*relative_pose.position;
    const UnitQuaternion q_ec = q_eb * q_bc;
    local_state.pose.orientation = q_ec;
    // Velocity of the component w.r.t body frame.
    const Eigen::Vector3d v_comp_body = body_state.velocity.linear + body_state.velocity.angular.cross(relative_pose.position);
    local_state.velocity.linear = q_bc.conjugate()*v_comp_body;
    local_state.velocity.angular = q_bc.conjugate()*body_state.velocity.angular;

    // Calculate the local environment.
    local_environment = environment;
    // Transform the relative wind from body axes to propeller axes
    local_environment.wind = q_ec * environment.wind;
    if (!std::isfinite(local_environment.wind.x()))
    {
        throw runtime_error("sytems.cpp: NaN value in localWind.x");
    }
    if (std::fabs(local_environment.wind.x()) > 1e+160)
    {
        throw runtime_error("systems.cpp: localWind.x over 1e+160");
    }
}

void Component::calc_model()
{
    calc_model_impl();
    ensure_log_channels();
}

Component::~Component()
{
    // Our registered values are pointers into this object, so the channel must
    // not outlive us: the next logging::log_frame() would read freed memory.
    // The epoch check means we only drop a channel we ourselves registered into
    // the current recording -- without it, an object left over from an earlier
    // recording would tear down the fresh channel of a live namesake.
    if (logging::is_enabled() && log_epoch_ == logging::epoch())
    {
        logging::drop_channel(get_name());
    }
}

void Component::calc_model_impl()
{
    // Run the gravity model.
    wrench_sum.wrenchGrav = gravity->getWrench(local_state, inertial);
    // Children will then run their own dynamics.
}

void Component::register_log_channels()
{
    logging::get_channel(get_name()).register_value("wrench_sum", &wrench_sum);
}

void Component::ensure_log_channels()
{
    if (!logging::is_enabled())
    {
        return;
    }
    // Register our channel for this recording the first time we are stepped
    // in it. enable() drops the previous channels and bumps the epoch, so a
    // fresh recording re-registers against the current member addresses.
    // Registering is all we do: logging::log_frame() is what writes the values.
    if (log_epoch_ != logging::epoch())
    {
        register_log_channels();
        log_epoch_ = logging::epoch();
    }
}

WrenchSum_t Component::rotate_wrenches() const
{
    const UnitQuaternion q_bc = relative_pose.orientation;
    auto wrenchSum_body = WrenchSum_t();

    Vector3d force_grav_body = q_bc * wrench_sum.wrenchGrav.force;
    Vector3d torque_grav_body = q_bc * wrench_sum.wrenchGrav.torque + relative_pose.position.cross(force_grav_body);
    const auto wrenchGrav_body = Wrench_t(force_grav_body, torque_grav_body);
    wrenchSum_body.wrenchGrav = wrenchGrav_body;

    Vector3d force_aero_body = q_bc * wrench_sum.wrenchAero.force;
    Vector3d torque_aero_body = q_bc * wrench_sum.wrenchAero.torque + relative_pose.position.cross(force_aero_body);
    const auto wrenchAero_body = Wrench_t(force_aero_body, torque_aero_body);
    wrenchSum_body.wrenchAero = wrenchAero_body;

    Vector3d force_prop_body = q_bc * wrench_sum.wrenchProp.force;
    Vector3d torque_prop_body = q_bc * wrench_sum.wrenchProp.torque + relative_pose.position.cross(force_prop_body);
    const auto wrenchProp_body = Wrench_t(force_prop_body, torque_prop_body);
    wrenchSum_body.wrenchProp = wrenchProp_body;

    return wrenchSum_body;
}

Inertial Component::rotate_inertia(const Eigen::Vector3d &r_com) const
{
    // Parallel-axis offset from the reference point r_com to this component.
    const auto x = relative_pose.position.x() - r_com.x();
    const auto y = relative_pose.position.y() - r_com.y();
    const auto z = relative_pose.position.z() - r_com.z();
    const auto mass = inertial.mass;
    UnitQuaternion q_bc = relative_pose.orientation;
    Eigen::Matrix3d inertia_rotated = q_bc.R_bi() * inertial.tensor * q_bc.R_bi().transpose();

    // Calculate parallel axis theorem.
    Eigen::Matrix3d tensor_body = inertia_rotated;
    tensor_body(0, 0) += mass*(y*y + z*z);
    tensor_body(1, 1) += mass*(x*x + z*z);
    tensor_body(2, 2) += mass*(x*x + y*y);
    tensor_body(0 , 1) += - mass*x*y;
    tensor_body(1 , 0) += - mass*x*y;
    tensor_body(0 , 2) += - mass*x*z;
    tensor_body(2 , 0) += - mass*x*z;
    tensor_body(1 , 2) += - mass*y*z;
    tensor_body(2 , 1) += - mass*y*z;

    auto inertia_body = Inertial(inertial.mass);
    inertia_body.tensor = tensor_body;

    return inertia_body;
}

///////////////////////////////////////////////////////////////////////////////
// Dynamic System
///////////////////////////////////////////////////////////////////////////////

DynamicSystem::DynamicSystem(stateType x_0_p, stateType u_0_p, double t_p) :
        x_0(x_0_p), u_0(u_0_p), t_0(t_p)
    {
        reset();
    }

DynamicSystem::DynamicSystem() :
        t_0(0)
    {
        reset();
    }

void DynamicSystem::reset(std::vector<double> x_0_p, std::vector<double> u_0_p, double t_p)
{
    x_0 = x_0_p;
    u_0 = u_0_p;
    t_0 = t_p;

    x = x_0;
    u = u_0;
    t = t_0;
}

void DynamicSystem::reset()
{
    reset(x_0, u_0, t_0);
}

stateType DynamicSystem::pack_odeint_state(const stateType u_p)
{
    stateType odeint_state;
    // Source - https://stackoverflow.com/a/3177252
    // Posted by Kirill V. Lyadvinsky
    // Retrieved 2026-06-24, License - CC BY-SA 2.5
    odeint_state.reserve(x.size() + u.size());
    odeint_state.insert( odeint_state.end(), x.begin(), x.end() );
    odeint_state.insert( odeint_state.end(), u_p.begin(), u_p.end() );

    return odeint_state;
}

void DynamicSystem::unpack_odeint_state(const stateType odeint_x, stateType &x_p, stateType &u_p)
{
    // Source - https://stackoverflow.com/a/421615
    // Posted by Greg Rogers
    // Retrieved 2026-06-24, License - CC BY-SA 2.5
    stateType::const_iterator u_begin = odeint_x.begin() + x_p.size();
    std::copy(odeint_x.begin(), u_begin, x_p.begin());
    std::copy(u_begin, odeint_x.end(), u_p.begin());
}

void DynamicSystem::step_dynamics(const std::vector<double> u, const double dt)
{
    // Only calculate dynamics if the system is not empty/static.
    if (x.size() > 0) {
        stateType odeint_state = pack_odeint_state(u);
        // Declare a lambda, because do_step requires a static function.
        auto sys = [this, u](const stateType &odeint_state, stateType &dxdt, double t) {
            stateType x_temp{x};
            stateType u_temp{u};
            unpack_odeint_state(odeint_state, x_temp, u_temp);
            dxdt = dynamics(x_temp, u_temp, t);
        };
        stepper.do_step(sys, odeint_state , t, dt);
        stateType throwaway_u{u};
        unpack_odeint_state(odeint_state, x, throwaway_u);
    }
    t += dt;
}

} // end namespace systems
} // end namespace last_letter_lib
