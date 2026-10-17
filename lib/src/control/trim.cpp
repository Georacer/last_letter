// The trimmer. This is the only translation unit that includes nlopt.hpp, so
// NLopt stays off the installed headers.

#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlopt.hpp>

#include "last_letter_lib/control/trim.hpp"
#include "last_letter_lib/math_utils.hpp"

namespace last_letter_lib {
namespace control {

using math_utils::EulerAngles;
using math_utils::UnitQuaternion;
using uav_utils::SimState_t;

namespace {

// The unknowns ahead of the controls, [phi, theta].
constexpr int attitude_unknowns{2};
// The residuals, [speedDot; rateDot].
constexpr int residuals{6};
// The first step of BOBYQA on the attitude, in rad.
constexpr double initial_step_attitude{0.1};
// The first step of BOBYQA on a control, as a fraction of its range.
constexpr double initial_step_control{0.1};

// The cost function sees the trimmer through this.
struct Problem
{
    UavModel *uav;
    const ControlMap *map;
    SimState_t base;
    TrimTarget target;
    Eigen::VectorXd weights;
    SettleOptions settle;
    int evaluations{0};
    // The first error of an evaluation. The NLopt wrapper drops the message of
    // an exception, so the cost function keeps it here and stops the solver.
    std::exception_ptr error;
};

// The state at the unknowns: the attitude from phi, theta and the heading,
// the body velocity from the inertial velocity, and the body rates from the
// yaw rate.
SimState_t build_state(const Problem &problem, double phi, double theta)
{
    SimState_t state = problem.base;
    const UnitQuaternion orientation(EulerAngles(phi, theta, problem.target.heading));
    state.pose.orientation = orientation;
    // The orientation turns a body vector into NED, so its conjugate turns
    // the target velocity into the body frame.
    const Eigen::Vector3d velocity_body = orientation.conjugate() * problem.target.velocity_ned;
    state.velocity.linear = velocity_body;
    const double psi_dot = problem.target.yaw_rate;
    state.velocity.angular = Eigen::Vector3d(-std::sin(theta) * psi_dot,
                                             std::sin(phi) * std::cos(theta) * psi_dot,
                                             std::cos(phi) * std::cos(theta) * psi_dot);
    return state;
}

// The residual [speedDot; rateDot] at the unknowns.
Eigen::Matrix<double, residuals, 1> residual(Problem &problem, const Eigen::VectorXd &unknowns)
{
    const SimState_t state = build_state(problem, unknowns(0), unknowns(1));
    const Eigen::VectorXd controls = unknowns.tail(unknowns.size() - attitude_unknowns);
    const Evaluation result = evaluate(*problem.uav, state, problem.map->to_input(controls), problem.settle);
    problem.evaluations++;
    Eigen::Matrix<double, residuals, 1> r;
    r << result.state_dot.speedDot, result.state_dot.rateDot;
    return r;
}

double cost(const std::vector<double> &x, std::vector<double> &gradient, void *data)
{
    (void)gradient; // BOBYQA is derivative free.
    auto &problem = *static_cast<Problem *>(data);
    try
    {
        const Eigen::VectorXd unknowns = Eigen::Map<const Eigen::VectorXd>(x.data(), static_cast<Eigen::Index>(x.size()));
        const Eigen::Matrix<double, residuals, 1> r = residual(problem, unknowns);
        return r.dot(problem.weights.cwiseProduct(r));
    }
    catch (...)
    {
        problem.error = std::current_exception();
        throw nlopt::forced_stop();
    }
}

bool is_converged(nlopt::result code)
{
    switch (code)
    {
    case nlopt::SUCCESS:
    case nlopt::STOPVAL_REACHED:
    case nlopt::FTOL_REACHED:
    case nlopt::XTOL_REACHED:
        return true;
    default:
        return false;
    }
}

} // namespace

struct Trimmer::Impl
{
    Impl(TrimOptions options_p) : options(std::move(options_p)) {}

    TrimOptions options;
    // Built in trim(), since its size follows the control map.
    nlopt::opt optimizer;
};

Trimmer::Trimmer(TrimOptions options) : impl_(std::make_unique<Impl>(std::move(options)))
{
}

Trimmer::~Trimmer() = default;

TrimPoint Trimmer::trim(UavModel &uav, const ControlMap &map, const TrimTarget &target)
{
    require_no_ground_reaction(uav);
    const TrimOptions &options = impl_->options;

    const int n_controls = map.size();
    const int n = attitude_unknowns + n_controls;
    const Eigen::VectorXd control_lower = map.lower_bounds();
    const Eigen::VectorXd control_upper = map.upper_bounds();
    if (control_lower.size() != n_controls || control_upper.size() != n_controls)
    {
        throw std::invalid_argument("Trimmer: the bounds of the control map do not match its size");
    }

    Problem problem;
    problem.uav = &uav;
    problem.map = &map;
    problem.target = target;
    problem.settle = options.settle;
    if (options.weights.size() == 0)
    {
        problem.weights = Eigen::VectorXd::Ones(residuals);
    }
    else if (options.weights.size() == residuals)
    {
        problem.weights = options.weights;
    }
    else
    {
        throw std::invalid_argument("Trimmer: TrimOptions::weights needs six entries or none");
    }

    // Start from the initial state of the recipe. That holds the geoid
    // coordinates, so the air density is the one of the recipe.
    uav.init();
    problem.base = uav.state;
    // A zero target position means the initial position of the recipe.
    if (!target.position.isZero(0.0))
    {
        const double climb = problem.base.pose.position.z() - target.position.z();
        problem.base.pose.position = target.position;
        problem.base.geoid.altitude += climb;
    }

    // The bounds. phi and theta stay inside plus or minus pi over 2.
    Eigen::VectorXd lower(n), upper(n), guess(n), step(n);
    lower << -M_PI / 2.0, -M_PI / 2.0, control_lower;
    upper << M_PI / 2.0, M_PI / 2.0, control_upper;

    // The initial guess is level, with the given controls or the copter default.
    Eigen::VectorXd initial_controls = options.initial_controls;
    if (initial_controls.size() == 0)
    {
        if (n_controls != 4)
        {
            throw std::invalid_argument("Trimmer: give TrimOptions::initial_controls for a map that is not a copter");
        }
        initial_controls = Eigen::Vector4d(0.0, 0.0, 0.0, 0.5);
    }
    if (initial_controls.size() != n_controls)
    {
        throw std::invalid_argument("Trimmer: TrimOptions::initial_controls does not match the size of the map");
    }
    guess << 0.0, 0.0, initial_controls;
    for (int i = 0; i < n; i++)
    {
        if (guess(i) < lower(i) || guess(i) > upper(i))
        {
            throw std::invalid_argument("Trimmer: the initial guess is outside the bounds");
        }
        step(i) = (i < attitude_unknowns) ? initial_step_attitude : initial_step_control * (upper(i) - lower(i));
    }

    nlopt::opt &optimizer = impl_->optimizer;
    optimizer = nlopt::opt(nlopt::LN_BOBYQA, static_cast<unsigned>(n));
    optimizer.set_lower_bounds(std::vector<double>(lower.data(), lower.data() + n));
    optimizer.set_upper_bounds(std::vector<double>(upper.data(), upper.data() + n));
    optimizer.set_initial_step(std::vector<double>(step.data(), step.data() + n));
    optimizer.set_ftol_abs(options.ftol_abs);
    optimizer.set_maxeval(options.max_evaluations);
    optimizer.set_min_objective(cost, &problem);

    std::vector<double> x(guess.data(), guess.data() + n);
    double best_cost = 0.0;
    nlopt::result code = nlopt::FAILURE;
    try
    {
        code = optimizer.optimize(x, best_cost);
    }
    catch (const nlopt::roundoff_limited &)
    {
        // NLopt still leaves the best point in x. The residual below says how good it is.
        code = nlopt::ROUNDOFF_LIMITED;
    }
    catch (const nlopt::forced_stop &)
    {
        if (problem.error)
        {
            std::rethrow_exception(problem.error);
        }
        throw;
    }

    // Leave the model at the trim point, state set and components settled.
    const Eigen::VectorXd unknowns = Eigen::Map<const Eigen::VectorXd>(x.data(), n);
    TrimPoint point;
    point.state = build_state(problem, unknowns(0), unknowns(1));
    point.controls = unknowns.tail(n_controls);
    point.input = map.to_input(point.controls);
    point.evaluations = problem.evaluations;
    point.residual = residual(problem, unknowns).norm();
    point.return_code = static_cast<int>(code);
    point.converged = is_converged(code);
    return point;
}

} // namespace control
} // namespace last_letter_lib
