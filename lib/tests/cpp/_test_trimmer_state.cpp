#include <chrono>
#include <iomanip>
// #include "prog_utils.hpp"
// #include "uav_utils.hpp"
#include "last_letter_lib/trimmer.hpp"
#include "Eigen/Eigen"

int main(int /* argc */, char * argv[])
{
    cout << "Testing TrimmerState class functionality" << endl;

    string uavName = argv[1];

    // double inf = std::numeric_limits<double>::infinity();

    TrimTrajectoryParameters_t trimParams;
    trimParams.Va = 10;
    trimParams.Gamma = 30*M_PI/180;
    trimParams.R = 200;

    TrimmerState trimmer(uavName);

    uint32_t loopNum=1000;
    // uint32_t loopNum=1;
    double seconds;

    cout << "Testing optimization time requirement." << endl;

    OptimResult_t result;
    auto t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        result = trimmer.findTrimState(trimParams);
    }
    auto t_end = chrono::steady_clock::now();
    seconds = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;

    cout << "Found minimum at\n" << vectorToString2(result.trimValues) << "\n with value = " << setprecision(10) << result.cost << endl;
    cout << "after " << trimmer.funCallCount << " function calls.\n";
    cout << "Optimization return code: " << result.returnCode << endl;
    cout << endl;

    cout << trimmer.printOptimalResult();

    cout << seconds << " second(s) elapsed for " << loopNum << " steps\n" << endl;

    cout << "Testing pyFindTrimState interface" << endl;
    double trajArray[3], resultArray[9];
    trajArray[0] = trimParams.Va;
    trajArray[1] = trimParams.Gamma;
    trajArray[2] = trimParams.R;

    trimmer.pyFindTrimState(trajArray, resultArray);

    printf("Got result:\n");
    for (int i=0; i<9; i++)
    {
        printf("%f", resultArray[i]); printf("\n");
    }
}
