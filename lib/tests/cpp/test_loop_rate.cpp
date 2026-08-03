#include <iostream>
#include <gtest/gtest.h>
#include <time.h>
#include <chrono>
#include <filesystem>

#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

TEST(TestLoopRate, TestLoopRate1)
{
    auto config = load_config_aircraft("skywalker_2013");
    SimState_t state = build_aircraft_state_from_config(config);
    state.pose.position.z() = -1000; // Lift the aircraft above the ground.
    UavModel uav("skywalker_2013");
    uav.initialize(config);

    uint32_t loopNum=10000;
    double t_steps, t_steps_nolog, t_derivs;

    cout << "Testing simulation loop rate" << endl;

    // Record all loopNum simulation steps to an MCAP log we can inspect
    // afterwards (PlotJuggler, or last_letter_lib.utils.log in Python).
    auto log_path = std::filesystem::absolute("test_loop_rate.mcap").string();
    uav.enable_logging(log_path);

    // Calculate step time requirements
    auto t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.step();
    }
    auto t_end = chrono::steady_clock::now();
    t_steps = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    // Stop recording (flushes + closes the MCAP) now that all loopNum steps are
    // done. The second timing loop below is not a simulation step: it re-runs
    // dynamics.calc_model() on a frozen state purely to benchmark it, so it is
    // intentionally left out of the log.
    uav.disable_logging();
    cout << "Wrote " << loopNum << "-step simulation log to " << log_path << endl;

    SimState_t newState;
    newState = uav.state;

    // Re-run the same number of steps from a reset state WITHOUT logging, so we
    // can compare against the logged run above and see the per-step logging
    // overhead. (init() resets the model to its initial state.)
    uav.init();
    t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.step();
    }
    t_end = chrono::steady_clock::now();
    t_steps_nolog = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    // Calculate dynamics calculation time requirements
    t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        uav.dynamics.calc_model(uav.state);
    }
    t_end = chrono::steady_clock::now();
    t_derivs = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    cout << "Initial State:\n" << endl;
    cout << "Position\n" << state.pose.position << endl;
    cout << "Coordinates\n" << state.geoid.latitude << "\n" << state.geoid.longitude << "\n" << state.geoid.altitude <<  endl;
    cout << "Orientation\n" << state.pose.orientation.vec() << "\n" << state.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << state.velocity.linear << endl;
    cout << "Angular Velocity\n" << state.velocity.angular << endl;
    cout << "Linear Acceleration\n" << state.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << state.acceleration.angular << endl;

    cout << "Propagated State:\n" << endl;
    cout << "Position\n" << newState.pose.position << endl;
    cout << "Coordinates\n" << newState.geoid.latitude << "\n" << newState.geoid.longitude << "\n" << newState.geoid.altitude << endl;
    cout << "Orientation\n" << newState.pose.orientation.vec() << "\n" << newState.pose.orientation.w() << endl;
    cout << "Linear Velocity\n" << newState.velocity.linear << endl;
    cout << "Angular Velocity\n" << newState.velocity.angular << endl;
    cout << "Linear Acceleration\n" << newState.acceleration.linear << endl;
    cout << "Angular Acceleration\n" << newState.acceleration.angular << endl;

    cout << t_steps << " second(s) elapsed for " << loopNum << " simulation steps (with logging)" << endl;
    cout << t_steps_nolog << " second(s) elapsed for " << loopNum << " simulation steps (no logging)" << endl;
    cout << "Logging overhead: " << (t_steps - t_steps_nolog) << " s total, "
         << (t_steps - t_steps_nolog) / loopNum * 1e6 << " us/step" << endl;
    cout << t_derivs << " second(s) elapsed for " << loopNum << " model derivative calculations" << endl;

}
