#include "yaml-cpp/yaml.h"

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/kinematics.hpp"
#include "last_letter_lib/sensors.hpp"

using namespace std;
using namespace last_letter_lib::programming_utils;

using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{

// Core class declarations

// Top UavModel object class
class UavModel : public Parametrized
{
public:
    ///////////
    // Variables
    SimState_t state;	 // Main simulation states
    Input input;		 // Normalized input to the model
    InputPwm_t PwmInput; // PWM input to the model
    double dt;			 // Simulation timestep in s
    double sim_time_{0}; // Elapsed simulation time in s (the logging clock)
    int chanReset;

    /////////
    // Members
    Kinematics kinematics;
    Dynamics dynamics;
    EnvironmentModel environmentModel;
    std::vector<std::shared_ptr<Sensor>> sensors;

private:
    // Initialization parameters
    Vector3d initPosition_, initVelLinear_, initVelAngular_, initCoordinates_;
    Quaterniond initOrientation_;
    int initChanReset_;
    Input initCtrlInput_;

    // Logging
    void log_state();       // Snapshot UavModel::state into its log channel.
    unsigned log_epoch_{0}; // Recording id we last registered channels for

    ///////////
    // Methods
public:
    // Constructor
    UavModel(string name);
    // Destructor
    ~UavModel() = default;
    void initialize(ParameterManager config) override;
    void initialize(const std::string yaml_str) override;
    void initialize_parameters() override;
    void update_parameters() override;

    void init(); // Initialize UavModel object
    void step(); // Perform simulation step

    // Change the integration timestep without re-running initialize(), which would
    // rebuild every sub-model and reset the state. Forwards to kinematics,
    // dynamics and the environment model, each of which retunes its own children.
    void set_dt(double dt_new);

    // Data logging. Call enable_logging() before the first step() so channels
    // are created with the MCAP sink attached.
    void enable_logging(const std::string &path = ""); // Start recording (empty path => timestamped filename).
    void disable_logging();                        // Stop recording and close the file.
    double get_time() const { return sim_time_; }  // Elapsed simulation time in s.

    SimState_t getState() { return state; } // Return the state of the Body Frame.
    void setInput(Input inputMsg);
    void setInputPwm(InputPwm_t inputMsg);
};

} // namespace last_letter_lib
