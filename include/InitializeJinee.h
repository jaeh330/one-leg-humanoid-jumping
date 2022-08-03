//
// Created by jaehyeong on 22. 6. 7.
//

#ifndef RAISIM_INITIALIZEJINEE_H
#define RAISIM_INITIALIZEJINEE_H

#include "PDController.h"


class InitializeJinee {
public:
    InitializeJinee(raisim::ArticulatedSystem *robot, raisim::World *robotWorld)
    {
        mRobot = robot;
        mWorld = robotWorld;
    }

    void InitJineeJointPosition(double position1, double position2, double position3, double position4);

    double deg2rad = 3.141592 / 180.0;
    double rad2deg = 180.0/3.141592;
    int iteration=0;

private:
    raisim::World *mWorld;
    raisim::ArticulatedSystem *mRobot;
};


#endif //RAISIM_INITIALIZEJINEE_H
