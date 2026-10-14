///////////////////////////////////////////////
// LinearCopter class related declarations //
///////////////////////////////////////////////

// A multicopter described by four linear models, one per axis, from a
// SystemID flight. The input is the ideal command into the ArduPilot mixer.
// The model lumps the propulsion and the aerodynamics of the airframe.

#pragma once

#include <Eigen/Eigen>

#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/body/body.hpp"

namespace last_letter_lib {
namespace body {

// bodyType 1.
class LinearCopter : public Body
{
public:
    LinearCopter(string name) : Body(name) {};
    ~LinearCopter() override {};
    void initialize_parameters() override;
    void update_parameters() override;
    void set_dt(double dt_p) override;
    // Store the four ideal inputs. They take effect at the next step.
    void setInput(const Input &input) override;
    // Recover the four ideal inputs from the motor PWM with the inverse mixer.
    void setInputPwm(const InputPwm_t &input) override;

    // Roll axis derivatives.
    double Yv{0}, Ylat{0}, Lv{0}, Lp{0}, Llat{0};
    // Pitch axis derivatives.
    double Xu{0}, Xlon{0}, Mu{0}, Mq{0}, Mlon{0};
    // Yaw axis derivatives.
    double Nr{0}, Nped{0}, Npedp{0};
    // Vertical axis derivatives and the hover throttle.
    double Zw{0}, Zcoll{0}, u_hover{0};
    // Actuator lag bandwidth per axis, in rad/s. Zero means no lag.
    double wlg_rol{0}, wlg_pit{0}, wlg_yaw{0}, wlg_thr{0};
    // Input channels. A value of -1 leaves that input at zero.
    int chanRoll{0}, chanPitch{1}, chanYaw{2}, chanThrottle{3};

    // Ideal inputs into the mixer. Roll, pitch and yaw are in -1 to 1,
    // throttle in 0 to 1.
    double u_rol{0}, u_pit{0}, u_yaw{0}, u_thr{0};
    // Actuator lag states.
    double d_rol{0}, d_pit{0}, d_yaw{0}, d_thr{0};

    // The inverse ArduPilot mixer, under the child parameters mixer/.
    ArduPilotMixerInverse mixer;

protected:
    void calc_model_impl() override;
    void register_log_channels() override;

private:
    // Refresh the lag factors 1 - exp(-wlg dt) after a change of dt or wlg.
    void update_lag_factors();
    double lag_rol_{1}, lag_pit_{1}, lag_yaw_{1}, lag_thr_{1};
};

} // namespace body
} // namespace last_letter_lib
