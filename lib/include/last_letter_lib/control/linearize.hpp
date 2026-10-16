/////////////////////////////////////////
// Linearizer related declarations //
/////////////////////////////////////////

// The linearizer takes the model, a trim point and a control map, and returns
// the Jacobians of the wrench accelerations by central differences. A lookup
// maps them onto the parameters of a LinearCopter.

#pragma once

#include <Eigen/Eigen>

#include "last_letter_lib/control/control_map.hpp"
#include "last_letter_lib/control/evaluate.hpp"
#include "last_letter_lib/control/trim.hpp"
#include "last_letter_lib/uav_model.hpp"

namespace last_letter_lib {
namespace control {

struct LinearizerOptions {
    double step_velocity{0.01};   // m/s
    double step_rate{0.01};       // rad/s
    double step_attitude{0.01};   // rad
    double step_control{0.01};    // ideal input
    bool include_gravity{true};
    bool include_coriolis{true};
    bool include_gyroscopic{true};
    SettleOptions settle;
};

struct LinearModel {
    Eigen::Matrix<double, 6, 3> A_v, A_w, A_att;  // rows [a; alpha], columns [u v w], [p q r], [phi theta psi]
    Eigen::MatrixXd B;                             // 6 by size() of the map
    LinearizerOptions options;                     // what the model was computed with
    TrimPoint trim;
};

struct LinearCopterCoefficients {
    double Yv, Ylat, Lv, Lp, Llat;
    double Xu, Xlon, Mu, Mq, Mlon;
    double Nr, Nped;
    double Zw, Zcoll, u_hover;
};

class Linearizer {
public:
    explicit Linearizer(LinearizerOptions options = {});
    LinearModel linearize(UavModel &uav, const ControlMap &map, const TrimPoint &trim);
    // Throws when any of the three term switches of model.options is on.
    static LinearCopterCoefficients to_linear_copter(const LinearModel &model);
};

} // namespace control
} // namespace last_letter_lib
