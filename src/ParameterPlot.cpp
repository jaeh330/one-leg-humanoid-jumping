#include "../include/ParameterPlot.h"

 void ParameterPlot::torquePlot(int figureNumber){
    matplotlibcpp::figure(figureNumber);
    matplotlibcpp::title("torque");
    matplotlibcpp::named_plot("hip_pitch",SimulationTime,Torque1Vector);
    matplotlibcpp::named_plot("knee_pitch",SimulationTime,Torque2Vector);
    matplotlibcpp::named_plot("ankle_pitch",SimulationTime,Torque3Vector);

     std::vector<double> torque1limit(SimulationTime.size(),30);
     std::vector<double> torque2limit(SimulationTime.size(),-30
     );

     matplotlibcpp::named_plot("Peak Torque",SimulationTime, torque1limit, "r--");
     matplotlibcpp::plot(SimulationTime, torque2limit, "r--");

     std::vector<double> normaltorque(SimulationTime.size(),13);
     std::vector<double> normaltorque2(SimulationTime.size(),-13);

     matplotlibcpp::named_plot("normal Torque",SimulationTime, normaltorque, "y--");
     matplotlibcpp::plot(SimulationTime, normaltorque2, "y--");

     matplotlibcpp::xlabel("time [s]");
     matplotlibcpp::ylabel("[N]");
     matplotlibcpp::legend();
     std::cout << "hjello" << std::endl;

}
void ParameterPlot::velocityPlot(int figureNumber, std::vector<double> simulationTime, std::vector<double> velocity1, std::vector<double> velocity2, std::vector<double> velocity3, Eigen::VectorXd desiredVelocity){
    std::vector<double> desiredVelocityVector1(simulationTime.size(),desiredVelocity[1]*rad2deg);
    std::vector<double> desiredVelocityVector2(simulationTime.size(),desiredVelocity[2]*rad2deg);
    std::vector<double> desiredVelocityVector3(simulationTime.size(),desiredVelocity[3]*rad2deg);

    matplotlibcpp::figure(figureNumber);
    matplotlibcpp::title("velocity");

    matplotlibcpp::plot(simulationTime, desiredVelocityVector1, "r--");
    matplotlibcpp::plot(simulationTime, desiredVelocityVector1, "r--");
    matplotlibcpp::plot(simulationTime, desiredVelocityVector1, "r--");

    matplotlibcpp::named_plot("velocity1",simulationTime,velocity1);
    matplotlibcpp::named_plot("velocity2",simulationTime,velocity2);
    matplotlibcpp::named_plot("velocity3",simulationTime,velocity3);


}
void ParameterPlot::positionPlot(int figureNumber, std::vector<double> simulationTime, std::vector<double> position1, std::vector<double> position2, std::vector<double> position3, Eigen::VectorXd desiredPosition){
    std::vector<double> desiredPositionVector1(simulationTime.size(),desiredPosition[1]*rad2deg);
    std::vector<double> desiredPositionVector2(simulationTime.size(),desiredPosition[2]*rad2deg);
    std::vector<double> desiredPositionVector3(simulationTime.size(),desiredPosition[3]*rad2deg);

    matplotlibcpp::figure(figureNumber);
    matplotlibcpp::title("position");

    matplotlibcpp::plot(simulationTime, desiredPositionVector1, "r--");
    matplotlibcpp::plot(simulationTime, desiredPositionVector2, "r--");
    matplotlibcpp::plot(simulationTime, desiredPositionVector3, "r--");

    matplotlibcpp::named_plot("position1",simulationTime,position1);
    matplotlibcpp::named_plot("position2",simulationTime,position2);
    matplotlibcpp::named_plot("position3",simulationTime,position3);
}

void ParameterPlot::pushBack(raisim::VecDyn position, raisim::VecDyn velocity , Eigen::VectorXd torque, int iteration ){

    Position1Vector.push_back(position[1]);
    Position2Vector.push_back(position[2]);
    Position3Vector.push_back(position[3]);
    Velocity1Vector.push_back(velocity[1]);
    Velocity2Vector.push_back(velocity[2]);
    Velocity3Vector.push_back(velocity[3]);
    Torque1Vector.push_back(torque[1]);
    Torque2Vector.push_back(torque[2]);
    Torque3Vector.push_back(torque[3]);

    SimulationTime.push_back(iteration*0.005);
}