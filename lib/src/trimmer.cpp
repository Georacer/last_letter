#include <math.h>
#include <iomanip>

#include <last_letter_lib/trimmer.hpp>

extern "C"
{
    TrimmerInput *trimmer_input_new(char *uavName)
    {
        TrimmerInput *newTrimmer = new TrimmerInput(uavName);
        return newTrimmer;
    }
    double *find_input_trim(TrimmerInput *trimmer, double *trimState)
    {
        static double result[6];
        trimmer->pyFindTrimInput(trimState, result);
        return result;
    }
    double print_optimal_input_result(TrimmerInput *trimmer)
    {
        trimmer->printOptimalResult(true);
        return 1;
    }

    TrimmerState *trimmer_state_new(char *uavName)
    {
        TrimmerState *newTrimmer = new TrimmerState(uavName);
        return newTrimmer;
    }
    double *find_state_trim(TrimmerState *trimmer, double *trimTrajectory)
    {
        static double result[9];
        trimmer->pyFindTrimState(trimTrajectory, result);
        return result;
    }
    double print_optimal_state_result(TrimmerState *trimmer)
    {
        trimmer->printOptimalResult(true);
        return 1;
    }
    bool set_parameter(TrimmerState *obj, ParamType_t paramType, char *name, double value)
    {
        return obj->uav->set_parameter(paramType, name, value);
    }
    void update_model(TrimmerState *obj)
    {
        obj->uav->update_model();
    }
}

namespace last_letter_lib
{
    namespace uav_utils
    {

        TrimmerState::TrimmerState(const string uavName) : opt(nlopt::LN_BOBYQA, 7),
                                                           initState(7, 0)
        {
            ParameterManager configs = loadModelConfig(uavName);
            uav = new UavModel(configs);

            // Optimization variables: See TRAJ_ARG_IDX

            std::vector<double> lb(7, 0); // Initialize lower bound vector
            lb[TRAJ_IDX_PHI] = -M_PI / 2; // Phi lower bound
            lb[TRAJ_IDX_AOA] = -M_PI / 4; // aoa lower bound
            lb[TRAJ_IDX_AOS] = -M_PI / 4; // aos lower bound
            lb[TRAJ_IDX_DELTAA] = -1;     // deltaa lower bound
            lb[TRAJ_IDX_DELTAE] = -1;     // deltae lower bound
            lb[TRAJ_IDX_DELTAT] = 0;      // deltat lower bound
            lb[TRAJ_IDX_DELTAR] = -1;     // deltar lower bound
            opt.set_lower_bounds(lb);

            std::vector<double> ub(7, 0); // Initialize upper bound vector
            ub[TRAJ_IDX_PHI] = M_PI / 2;  // Phi upper bound
            ub[TRAJ_IDX_AOA] = M_PI / 4;  // aoa upper bound
            ub[TRAJ_IDX_AOS] = M_PI / 4;  // aos upper bound
            ub[TRAJ_IDX_DELTAA] = 1;      // deltaa upper bound
            ub[TRAJ_IDX_DELTAE] = 1;      // deltae upper bound
            ub[TRAJ_IDX_DELTAT] = 1;      // deltat upper bound
            ub[TRAJ_IDX_DELTAR] = 1;      // deltar upper bound
            opt.set_upper_bounds(ub);

            opt.set_min_objective(objFunWrapper, this);

            // opt.set_xtol_abs(1e-2);
            // opt.set_xtol_rel(1e-4);
            // opt.set_ftol_abs(1e-7);
            opt.set_ftol_rel(1e-4);
        }
        TrimmerState::~TrimmerState()
        {
        }

        void TrimmerState::setInitState(const vector<double> inputVect)
        {
            initState = inputVect;
        }

        void TrimmerState::resetFunCallCount()
        {
            funCallCount = 0;
        }

        // Calculate the optimization error cost, based on the optimized input and the corresponding derivative
        double TrimmerState::calcCost(const SimState_t state, const Derivatives_t stateDer, const Input input)
        {
            // State derivative weights
            double derSpeedWeight = 100;
            double derRateWeight = 100;
            // State weights
            double aosWeight = 1;
            // Input weights
            Eigen::Matrix<double, 4, 4> inputWeight;
            inputWeight.setZero();
            inputWeight.diagonal() << 0, 0, 1, 0;

            /////////////////////////
            // State derivative error costs
            // The position and orientation derivatives are not taken into account, because they are designed to be optimal
            // and the optimized input vector does not affect them anyway.

            // Calculate velocity derivative error
            double speedDerTerm = stateDer.speedDot.transpose() * derSpeedWeight * stateDer.speedDot;

            // Calculate rate derivative error
            double rateDerTerm = stateDer.rateDot.transpose() * derRateWeight * stateDer.rateDot;

            ////////////////////
            // State error costs
            // Calculate output error
            Vector3d airdata = getAirData(state.velocity.linear);

            double aosTerm = airdata(2) * aosWeight * airdata(2);

            //////////////
            // Input costs
            Eigen::Vector4d inputVec;
            inputVec << input.value[0], input.value[1], input.value[2], input.value[3];
            double inputTerm = inputVec.transpose() * inputWeight * inputVec;

            /////////
            // Sum up
            return +speedDerTerm + rateDerTerm + aosTerm + inputTerm;
        }

        SimState_t TrimmerState::buildStateFromArgs(const vector<double> optim_arg)
        {
            SimState_t state;

            // Set height
            state.pose.position(0) = 0;
            state.pose.position(1) = 0;
            state.pose.position(2) = -10; // Lift from the ground the aircraft a bit to avoid ground reactions

            // Set linear velocities
            Vector3d airdata;
            airdata << targetTrajectory.Va, optim_arg[TRAJ_IDX_AOA], optim_arg[TRAJ_IDX_AOS];
            Vector3d velLinear = getVelocityFromAirdata(airdata);
            state.velocity.linear = velLinear;

            // Set orientation
            Vector3d euler;
            euler(0) = optim_arg[TRAJ_IDX_PHI];
            // euler(1) = optim_arg[TRAJ_IDX_THETA];
            euler(1) = targetTrajectory.Gamma + airdata(1);
            euler(2) = 0;
            Quaterniond orientation = euler2quat(euler).conjugate(); // euler2quat produces q_eb
            state.pose.orientation = orientation;                    // but model needs q_be

            // Set angular velocities
            double targetPsiDot = targetTrajectory.Va * cos(targetTrajectory.Gamma) / targetTrajectory.R;
            Vector3d targetEulerDot(0, 0, targetPsiDot);
            Vector3d angularRates = getAngularRatesFromEulerDerivatives(euler, targetEulerDot);
            state.velocity.angular = angularRates;

            // If your motors have state as well (RPM) you need to set it here again
            return state;
        }

        Input TrimmerState::buildInputFromArgs(const vector<double> optim_arg)
        {
            Input input;
            // Setup model input
            input.value[0] = optim_arg[TRAJ_IDX_DELTAA];
            input.value[1] = optim_arg[TRAJ_IDX_DELTAE];
            input.value[2] = optim_arg[TRAJ_IDX_DELTAT];
            input.value[3] = optim_arg[TRAJ_IDX_DELTAR];

            return input;
        }

        // Wrapper to the cost function. Accepts concatenated state and input x under optimization and also passes the trimState parameter to the cost function
        double TrimmerState::costWrapper(const vector<double> &optim_arg, vector<double> & /* grad */)
        {
            ++funCallCount;

            // Setup model state
            SimState_t state = buildStateFromArgs(optim_arg);
            Input input = buildInputFromArgs(optim_arg);

            // Simulate the optimization arguments
            uav->setState(state);
            uav->setInput(input);
            uav->step();

            Derivatives_t stateDot = uav->kinematics.stateDot;

            // Calculate the optimization cost
            return calcCost(state, stateDot, input);
        }

        double TrimmerState::objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr)
        {
            TrimmerState *obj = static_cast<TrimmerState *>(trimmerObjPtr);
            return obj->costWrapper(u, grad);
        }

        OptimResult_t TrimmerState::findTrimState(const TrimTrajectoryParameters_t p_trimParams)
        {
            resetFunCallCount();

            // Build the initializing input
            vector<double> vectorInit(7, 0);
            vectorInit[TRAJ_IDX_DELTAT] = 0.5; // Set initial throttle to half
            setInitState(vectorInit);

            targetTrajectory = p_trimParams;

            result.trimValues = initState;

            try
            {
                nlopt::result returnCode = opt.optimize(result.trimValues, result.cost);

                result.returnCode = returnCode;
                result.success = true;

                trimState = buildStateFromArgs(result.trimValues);
                trimInput = buildInputFromArgs(result.trimValues);
            }
            catch (exception &e)
            {
                cout << "nlopt failed: " << e.what() << endl;
                result.success = false;
            }

            return result;
        }

        // Simplified wrapper for calling via ctypes python API
        // Input: a double array[3] holding the trim values for (in order):
        //  Va, gamma, R
        // Ouptut: a double array[14] holding:
        //  Positions 0-2: the trim state (phi, aoa, aos)
        //  Positions 3-6: the trim control inputs (da, de, dt, dr)
        //  Position 7: the optimization cost
        //  Position 8: the success flag (0/1)
        void TrimmerState::pyFindTrimState(double *trimParamArray, double *result)
        {
            TrimTrajectoryParameters_t trimParams;
            trimParams.Va = trimParamArray[0];
            trimParams.Gamma = trimParamArray[1];
            trimParams.R = trimParamArray[2];

            OptimResult_t resultStruct = findTrimState(trimParams);

            for (int i = 0; i < 7; i++)
            {
                result[i] = resultStruct.trimValues[i];
            }
            result[7] = resultStruct.cost;
            result[8] = (double)resultStruct.success;
        }

        string TrimmerState::printOptimalResult(bool verbose)
        {
            ostringstream oss;
            // Re-run the optimal state and input
            uav->setState(trimState);
            uav->setInput(trimInput);
            uav->step();

            Vector3d eulerAngles = quat2euler(uav->state.pose.orientation);
            Vector3d eulerDot = getEulerDerivatives(eulerAngles, uav->state.velocity.angular);

            oss << "Optimal calculated propagated state:\n";
            oss << "position:\n"
                << vectorToString2(uav->state.pose.position, "\n");
            oss << "orientation:\n"
                << vectorToString2(eulerAngles, "\n");
            oss << "linearVel:\n"
                << vectorToString2(uav->state.velocity.linear, "\n");
            oss << "angularVel:\n"
                << vectorToString2(uav->state.velocity.angular, "\n");
            oss << endl;
            oss << "Optimal state derivatives:\n";
            oss << "posDot:\n"
                << vectorToString2(uav->kinematics.stateDot.posDot, "\n");
            oss << "AngleDot:\n"
                << vectorToString2(eulerDot, "\n");
            oss << "speedDot:\n"
                << vectorToString2(uav->kinematics.stateDot.speedDot, "\n");
            oss << "rateDot:\n"
                << vectorToString2(uav->kinematics.stateDot.rateDot, "\n");
            oss << "Trim input:\n";
            for (int i = 0; i < 4; i++)
            {
                oss << uav->input.value[i] << ", ";
            }
            oss << endl;
            oss << endl;

            Eigen::Vector3d airdata = getAirData(trimState.velocity.linear);
            double gamma = eulerAngles(1) - airdata(1);
            double R = airdata(0) * cos(gamma) / eulerDot(2);

            cout << setprecision(3);
            cout << "Resulting trajectory:\n";
            cout << "Airspeed:\t" << airdata(0) << "(" << targetTrajectory.Va << ")\n";
            cout << "Gamma:\t" << gamma * 180 / M_PI << "(" << targetTrajectory.Gamma * 180 / M_PI << ")\n";
            cout << "R:\t" << R << "(" << targetTrajectory.R << ")\n";
            cout << "AoS:\t" << airdata(2) << "\n";
            oss << endl;

            if (verbose)
            {
                std::cout << oss.str();
            }

            return oss.str();
        }

        ////////////////////////////////
        // TrimmerInput class definition
        ////////////////////////////////

        TrimmerInput::TrimmerInput(const string uavName) : opt(nlopt::LN_BOBYQA, 4),
                                                           // opt(nlopt::LN_COBYLA, 4);
                                                           // opt(nlopt::LN_PRAXIS, 4); // Known working
                                                           initInput(4, 0)
        {
            ParameterManager configs = loadModelConfig(uavName);
            uav = new UavModel(configs);

            std::vector<double> lb(4, -1);
            lb[2] = 0;
            opt.set_lower_bounds(lb);

            std::vector<double> ub(4, 1);
            opt.set_upper_bounds(ub);

            // Set initial input value
            vector<double> inputVect{0, 0, 0.5, 0};
            setInitInput(inputVect);

            // Calculate trim derivatives

            opt.set_min_objective(objFunWrapper, this);

            // opt.set_xtol_abs(1e-2);
            opt.set_ftol_abs(0.001);
            // opt.set_xtol_rel(1e-4);
        }

        TrimmerInput::~TrimmerInput()
        {
        }

        void TrimmerInput::setInitInput(const vector<double> inputVect)
        {
            initInput = inputVect;
        }

        void TrimmerInput::resetFunCallCount()
        {
            funCallCount = 0;
        }

        // Calculate the full trim state and its derivative given the 6 independent trim variables
        TrimState_t TrimmerInput::calcTrimState(const TrimStateParameters_t trimParams)
        {
            double trim_phi = trimParams.phi;
            double trim_theta = trimParams.theta;
            double trim_Va = trimParams.Va;
            double trim_alpha = trimParams.alpha;
            double trim_beta = trimParams.beta;
            double trim_r = trimParams.r;

            // Calculate dependent state elements
            double k = trim_r / (cos(trim_phi) * cos(trim_theta));
            double trim_p = -k * sin(trim_theta);
            double trim_q = k * sin(trim_phi) * cos(trim_theta);

            TrimState_t newTrimState;
            newTrimState.trimState.position = Vector3d(0, 0, -100);
            newTrimState.trimState.euler = Vector3d(trim_phi, trim_theta, 0);
            Vector3d velocity = getVelocityFromAirdata(Vector3d(trim_Va, trim_alpha, trim_beta));
            newTrimState.trimState.linearVel = velocity;
            newTrimState.trimState.angularVel = Vector3d(trim_p, trim_q, trim_r);

            // Calculate derived quantities
            double gamma = trim_theta - trim_alpha;

            newTrimState.trimDerivatives.position = Vector3d(0, 0, -trim_Va * sin(gamma));
            newTrimState.trimDerivatives.euler = Vector3d(0, 0, k); // k=Va/R*cos(gamm)
            newTrimState.trimDerivatives.linearVel = Vector3d::Zero();
            newTrimState.trimDerivatives.angularVel = Vector3d::Zero();

            return newTrimState;
        }

        // Calculate the optimization error cost, based on the optimized input and the corresponding derivative
        double TrimmerInput::calcCost(const Derivatives_t stateDer, Eigen::Vector4d input)
        {
            // The cost function should ideally check the u and v components of the linear velocity.
            // But since this model deals with body-frame forces, instead of wind-frame forces (as the Python model/trimmer does)
            // the moments cannot affect alpha and beta. As a result, it is numerically very difficult for thrust input to fix all
            // of the u and w velocities.
            // Thus, for now we leave only airspeed in the juristiction of thrust and hope that alpha and beta can stay close to their
            // trimmed values, since control inputs will stabilize angular velocities
            double derSpeedWeight = 100;
            double derAngleWeight = 10000;
            double derRateWeight = 100;
            double inputWeight = 10;

            double speedDerTerm = stateDer.speedDot.transpose() * derSpeedWeight * stateDer.speedDot;
            // double speedDerTerm = stateDer.speedDot[0]*derErrorWeight*stateDer.speedDot[0];
            double angleDerTerm = stateDer.quatDot.coeffs().segment<2>(0).transpose() * derAngleWeight * stateDer.quatDot.coeffs().segment<2>(0);
            double rateDerTerm = stateDer.rateDot.transpose() * derRateWeight * stateDer.rateDot;
            // The position and orientation derivatives are not taken into account, because they are designed to be optimal
            // and the optimized input vector does not affect them anyway.

            double inputTerm = input.transpose() * inputWeight * input;

            return speedDerTerm + angleDerTerm + rateDerTerm + inputTerm;
        }

        // Wrapper to the cost function. Accepts input u under optimization and also passes the trimState parameter to the cost function
        double TrimmerInput::costWrapper(const vector<double> &u, vector<double> & /* grad */, TrimState_t trimState)
        {
            ++funCallCount;

            Input input = convertInput4ll(u);
            SimState_t state = convertState4ll(trimState.trimState);

            // If your motors have state as well (RPM) you need to set it here again
            uav->setState(state);
            uav->setInput(input);
            uav->step();

            Derivatives_t stateDot = uav->kinematics.stateDot;

            Eigen::Vector4d inputVect;
            inputVect[0] = input.value[0];
            inputVect[1] = input.value[1];
            inputVect[2] = input.value[2];
            inputVect[3] = input.value[3];
            return calcCost(stateDot, inputVect);
        }

        double TrimmerInput::objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr)
        {
            TrimmerInput *obj = static_cast<TrimmerInput *>(trimmerObjPtr);
            return obj->costWrapper(u, grad, obj->trimState);
        }

        OptimResult_t TrimmerInput::findTrimInput(const TrimStateParameters_t p_trimParams)
        {
            resetFunCallCount();

            // extract the trim states and state derivatives
            trimState = calcTrimState(p_trimParams);

            // Copy the initializing input
            result.trimValues = initInput;

            try
            {
                nlopt::result returnCode = opt.optimize(result.trimValues, result.cost);

                result.returnCode = returnCode;
                result.success = true;
            }
            catch (exception &e)
            {
                cout << "nlopt failed: " << e.what() << endl;
                result.success = false;
            }

            return result;
        }

        // Simplified wrapper for calling via ctypes python API
        // Input: a double array[6] holding the trim values for (in order):
        //  phi, theta, Va, alpha, beta, r
        // Ouptut: a double array[6] holding:
        //  Positions 0-3: the trim control inputs
        //  Position 4: the optimization cost
        //  Position 5: the success flag (0/1)
        void TrimmerInput::pyFindTrimInput(double *trimParamArray, double *result)
        {
            TrimStateParameters_t trimParams;
            trimParams.phi = trimParamArray[0];
            trimParams.theta = trimParamArray[1];
            trimParams.Va = trimParamArray[2];
            trimParams.alpha = trimParamArray[3];
            trimParams.beta = trimParamArray[4];
            trimParams.r = trimParamArray[5];

            OptimResult_t resultStruct = findTrimInput(trimParams);

            // double result[6];
            result[0] = resultStruct.trimValues[0];
            result[1] = resultStruct.trimValues[1];
            result[2] = resultStruct.trimValues[2];
            result[3] = resultStruct.trimValues[3];
            result[4] = resultStruct.cost;
            result[5] = (double)resultStruct.success;
        }

        string TrimmerInput::printOptimalResult(bool verbose)
        {
            ostringstream oss;
            // Re-run the optimal state and input
            uav->setState(convertState4ll(trimState.trimState));
            uav->setInput(convertInput4ll(result.trimValues));
            uav->step();

            oss << "Optimal calculated propagated state:\n";
            oss << "position:\n"
                << vectorToString2(uav->state.pose.position, "\n");
            oss << "orientation:\n"
                << vectorToString2(uav->state.pose.orientation, "\n");
            oss << "linearVel:\n"
                << vectorToString2(uav->state.velocity.linear, "\n");
            oss << "angularVel:\n"
                << vectorToString2(uav->state.velocity.angular, "\n");
            oss << endl;
            oss << "Optimal state derivatives:\n";
            oss << "posDot:\n"
                << vectorToString2(uav->kinematics.stateDot.posDot, "\n");
            oss << "speedDot:\n"
                << vectorToString2(uav->kinematics.stateDot.speedDot, "\n");
            oss << "rateDot:\n"
                << vectorToString2(uav->kinematics.stateDot.rateDot, "\n");
            oss << endl;

            if (verbose)
            {
                std::cout << oss.str();
            }

            return oss.str();
        }

        // Convert State_t to SimState_t for passing to UavModel
        SimState_t convertState4ll(const State_t p_state)
        {
            SimState_t state;
            state.pose.position = p_state.position;
            state.pose.orientation = euler2quat(p_state.euler).conjugate();
            state.velocity.linear = p_state.linearVel;
            state.velocity.angular = p_state.angularVel;
            return state;
        }

        // Convert vector input to Input for passing to UavModel
        Input convertInput4ll(const vector<double> &u)
        {
            Input input;
            input.value[0] = u[0];
            input.value[1] = u[1];
            input.value[2] = u[2];
            input.value[3] = u[3];
            return input;
        }

    } // namespace uav_utils
} // namespace last_letter_lib
