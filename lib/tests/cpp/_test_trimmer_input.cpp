#include <chrono>
#include <iomanip>
// #include "prog_utils.hpp"
// #include "uav_utils.hpp"
#include "last_letter_lib/trimmer.hpp"

int main(int /* argc */, char * argv[])
{
    cout << "Testing TrimmerInput class functionality" << endl;

    string uavName = argv[1];

    TrimStateParameters_t trimParams;
    trimParams.phi = 0;
    trimParams.theta = 0*M_PI/180;
    trimParams.Va = 10;
    trimParams.alpha = 0*M_PI/180;
    trimParams.beta = 0;
    trimParams.r = 0*M_PI/180;

    TrimmerInput trimmer(uavName);

    // vector<double> initInput {0, 0, 0.5, 0};
    // trimmer.setInitInput(initInput);

    TrimState_t trimState = trimmer.trimState;
    cout << "Generated trim state:\n";
    cout << "position:\n" << vectorToString2(trimState.trimState.position, "\n");
    cout << "euler:\n" << vectorToString2(trimState.trimState.euler, "\n");
    cout << "linearVel:\n" << vectorToString2(trimState.trimState.linearVel, "\n");
    cout << "angularVel:\n" << vectorToString2(trimState.trimState.angularVel, "\n");
    cout << endl;

    cout << "Desired state derivatives:\n";
    cout << "posDot:\n" << vectorToString2(trimState.trimDerivatives.position, "\n");
    cout << "eulDot:\n" << vectorToString2(trimState.trimDerivatives.euler, "\n");
    cout << "speedDot:\n" << vectorToString2(trimState.trimDerivatives.linearVel, "\n");
    cout << "rateDot:\n" << vectorToString2(trimState.trimDerivatives.angularVel, "\n");
    cout << endl;

    OptimResult_t result;

    uint32_t loopNum=1000;
    // uint32_t loopNum=1;
    double seconds;

    cout << "Testing optimization time requirement." << endl;

    auto t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        result = trimmer.findTrimInput(trimParams);
    }
    auto t_end = chrono::steady_clock::now();
    seconds = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    cout << "Found minimum at\n" << vectorToString2(result.trimValues) << "\n with value = " << setprecision(10) << result.cost << endl;
    cout << "after " << trimmer.funCallCount << " function calls.\n";
    cout << "Optimization return code: " << result.returnCode << endl;
    cout << endl;

    cout << trimmer.printOptimalResult();

    cout << seconds << " second(s) elapsed for " << loopNum << " steps" << endl;

    cout << "Testing pyFindTrimInput interface" << endl;
    double stateArray[6], resultArray[6];
    stateArray[0] = trimParams.phi;
    stateArray[1] = trimParams.theta;
    stateArray[2] = trimParams.Va;
    stateArray[3] = trimParams.alpha;
    stateArray[4] = trimParams.beta;
    stateArray[5] = trimParams.r;

    trimmer.pyFindTrimInput(stateArray, resultArray);

    printf("Got result:\n");
    printf("%f", resultArray[0]); printf("\n");
    printf("%f", resultArray[1]); printf("\n");
    printf("%f", resultArray[2]); printf("\n");
    printf("%f", resultArray[3]); printf("\n");
    printf("%f", resultArray[4]); printf("\n");
    printf("%f", resultArray[5]); printf("\n");
}
