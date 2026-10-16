// The trimmer. This is the only translation unit that includes nlopt.hpp, so
// NLopt stays off the installed headers. The trim logic lands with step 5 of
// docs/plans/linearization.md. Until then only the pimpl exists, which proves
// that NLopt links into the library.

#include <stdexcept>
#include <utility>

#include <nlopt.hpp>

#include "last_letter_lib/control/trim.hpp"

namespace last_letter_lib {
namespace control {

// The six unknowns of a copter trim, [phi, theta, roll, pitch, yaw, throttle].
static constexpr unsigned copter_unknowns{6};

struct Trimmer::Impl
{
    explicit Impl(TrimOptions options_p)
        : options(std::move(options_p)), optimizer(nlopt::LN_BOBYQA, copter_unknowns)
    {
    }

    TrimOptions options;
    nlopt::opt optimizer;
};

Trimmer::Trimmer(TrimOptions options) : impl_(std::make_unique<Impl>(std::move(options)))
{
}

Trimmer::~Trimmer() = default;

TrimPoint Trimmer::trim(UavModel &uav, const ControlMap &map, const TrimTarget &target)
{
    (void)uav;
    (void)map;
    (void)target;
    throw std::logic_error("Trimmer::trim() is not implemented yet");
}

} // namespace control
} // namespace last_letter_lib
