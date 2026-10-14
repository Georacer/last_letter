/////////////////////////////////////
// Body class related declarations //
/////////////////////////////////////

// A body is a Component that is neither a wing nor a motor. A recipe declares
// its bodies in body.yaml, which loadModelConfig places under dynamics/body/.

#pragma once

#include <memory>

#include <Eigen/Eigen>

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/systems.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {
namespace body {

using last_letter_lib::programming_utils::ParameterManager;
using last_letter_lib::uav_utils::Input;
using last_letter_lib::uav_utils::InputPwm_t;

// The base body, bodyType 0. It outputs only its gravity wrench.
class Body : public systems::Component
{
public:
    Body(string name) : Component(name) {};
    ~Body() override {};
    // Store a new control input. The base body reads no input.
    virtual void setInput(const Input &input) { (void)input; }
    // Store a new control input in PWM microseconds. The base body reads no input.
    virtual void setInputPwm(const InputPwm_t &input) { (void)input; }
};

// Build a body from its parameters. The bodyType key selects the class.
std::unique_ptr<Body> buildBody(ParameterManager config);

} // namespace body
} // namespace last_letter_lib
