//
// Created by jaehyeong on 22. 6. 6.
//

#ifndef RAISIM_PDCONTROLLER_H
#define RAISIM_PDCONTROLLER_H

#include <iostream>
#include "raisim/RaisimServer.hpp"
#include "Robot.h"

class PDController {
public:
    PDController(raisim::ArticulatedSystem *robot, raisim::World *robotWorld)
    {
        mRobot = robot;
        mWorld = robotWorld;
    }
    Eigen::VectorXd torque = Eigen::VectorXd(4);
    raisim::VecDyn position = raisim::VecDyn(4);
    raisim::VecDyn velocity = raisim::VecDyn(4);
    Eigen::VectorXd desiredPosition = Eigen::VectorXd(4);
    Eigen::VectorXd desiredVelocity = Eigen::VectorXd(4);
    double Kp;
    double Kd;

    void setDesiredPosition(double position1, double position2, double position3);
    void setDesiredVelocity(double velocity1, double velocity2, double velocity3);
    void setPDGain(double PGain, double DGain);
    void computeControllerInput();




    double deg2rad = 3.141592 / 180.0;
    double rad2deg = 180.0/3.141592;

private:

    raisim::World *mWorld;
    raisim::ArticulatedSystem *mRobot;
};


#endif //RAISIM_PDCONTROLLER_H
