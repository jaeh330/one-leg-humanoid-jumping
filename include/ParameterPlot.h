#include <iostream>
#include "raisim/RaisimServer.hpp"
#include <matplotlibcpp.h>
#include <vector>
#include "PDController.h"

class ParameterPlot  {
public:
    double rad2deg = 180.0 / 3.141592;
    double deg2rad = 3.141592 / 180.0;
    double enc2rad = 2 * 3.141592 / 65535;
    std::vector<double> SimulationTime;


    void torquePlot(int figureNumber);
    void velocityPlot(int figureNumber, std::vector<double> simulationTime, std::vector<double> velocity1, std::vector<double> velocity2, std::vector<double> velocity3, Eigen::VectorXd desiredVelocity);
    void positionPlot(int figureNumber, std::vector<double> simulationTime, std::vector<double> position1, std::vector<double> position2, std::vector<double> position3, Eigen::VectorXd desiredPosition);
    void pushBack(raisim::VecDyn position, raisim::VecDyn velocity , Eigen::VectorXd torque, int iteration );

private:
    std::vector<double> Position1Vector;
    std::vector<double> Position2Vector;
    std::vector<double> Position3Vector;
    std::vector<double> Velocity1Vector;
    std::vector<double> Velocity2Vector;
    std::vector<double> Velocity3Vector;
    std::vector<double> Torque1Vector;
    std::vector<double> Torque2Vector;
    std::vector<double> Torque3Vector;
};