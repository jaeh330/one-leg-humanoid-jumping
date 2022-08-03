//
// Created by jaehyeong on 22. 6. 7.
//

#include "../include/InitializeJinee.h"

void InitializeJinee::InitJineeJointPosition(double position1, double position2, double position3, double position4){
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = position1;
    initialJointPosition[1] = position2*deg2rad;
    initialJointPosition[2] = position3*deg2rad;
    initialJointPosition[3] = position4*deg2rad;
    mRobot->setGeneralizedCoordinate(initialJointPosition);
}
