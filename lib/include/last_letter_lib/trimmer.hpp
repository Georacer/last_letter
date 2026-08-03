#ifndef TRIMMER_
#define TRIMMER_

#include <Eigen/Eigen>
#include <nlopt.hpp>

#include "uav_model.hpp"

using namespace std;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{
    namespace uav_utils
    {

        struct State_t
        {
            Vector3d position;
            Vector3d euler;
            Vector3d linearVel;
            Vector3d angularVel;
        };

        struct TrimTrajectoryParameters_t
        {
            double Va;
            double Gamma;
            double R;
        };

        // Input parameters which serve as independent variables defining the trim state
        struct TrimStateParameters_t
        {
            double phi;
            double theta;
            double Va;
            double alpha;
            double beta;
            double r;
        };

        struct TrimState_t
        {
            State_t trimState;
            State_t trimDerivatives;
        };

        struct OptimResult_t
        {
            vector<double> trimValues;
            double cost;
            int returnCode;
            bool success;
        };

        // Indices of optimization variables within the trajectory optimizer argument vector
        enum TRAJ_ARG_IDX
        {
            TRAJ_IDX_PHI = 0,
            // TRAJ_IDX_THETA,
            TRAJ_IDX_AOA,
            TRAJ_IDX_AOS,
            // TRAJ_IDX_P,
            // TRAJ_IDX_Q,
            // TRAJ_IDX_R,
            TRAJ_IDX_DELTAA,
            TRAJ_IDX_DELTAE,
            TRAJ_IDX_DELTAT,
            TRAJ_IDX_DELTAR
        };

        class TrimmerState
        {
        public:
            UavModel *uav;
            uint funCallCount = 0;
            nlopt::opt opt; // The nlopt optimizer object
            OptimResult_t result;
            vector<double> initState;

            SimState_t trimState;
            Input trimInput;

            TrimTrajectoryParameters_t targetTrajectory;

            TrimmerState(const string uavName);
            ~TrimmerState();
            void setInitState(vector<double>);
            void resetFunCallCount();
            double calcCost(const SimState_t state, const Derivatives_t stateDer, const Input input);
            SimState_t buildStateFromArgs(const vector<double> optim_arg);
            Input buildInputFromArgs(const vector<double> optim_arg);
            double costWrapper(const vector<double> &u, vector<double> &grad);
            static double objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr);
            OptimResult_t findTrimState(const TrimTrajectoryParameters_t);
            void pyFindTrimState(double *trimParamArray, double *result);
            string printOptimalResult(bool verbose = false);
        };

        class TrimmerInput
        {
        public:
            UavModel *uav;
            uint funCallCount = 0;
            nlopt::opt opt; // The nlopt optimizer object
            OptimResult_t result;
            vector<double> initInput;

            TrimState_t trimState;

            TrimmerInput(const string uavName);
            ~TrimmerInput();
            void setInitInput(vector<double>);
            void resetFunCallCount();
            TrimState_t calcTrimState(const TrimStateParameters_t);
            double calcCost(const Derivatives_t stateDer, Eigen::Vector4d input);
            double costWrapper(const vector<double> &u, vector<double> &grad, TrimState_t trimState);
            static double objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr);
            OptimResult_t findTrimInput(const TrimStateParameters_t);
            void pyFindTrimInput(double *trimParamArray, double *result);
            string printOptimalResult(bool verbose = false);
        };

        SimState_t convertState4ll(const State_t p_state);
        Input convertInput4ll(const vector<double> &u);
    } // namespace uav_utils
} // namespace last_letter_lib

#endif
