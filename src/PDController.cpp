//
// Created by jaehyeong on 22. 6. 6.
//
#include "../include/PDController.h"

void PDController::setDesiredPosition(double position1, double position2, double position3){
    desiredPosition << 0.0, position1*deg2rad, position2*deg2rad, position3*deg2rad;
}

void PDController::setDesiredVelocity(double velocity1, double velocity2, double velocity3){
    desiredVelocity << 0.0, velocity1*deg2rad, velocity2*deg2rad, velocity3*deg2rad;
}

void PDController::setPDGain(double PGain, double DGain) {
    this->Kp=PGain;
    this->Kd=DGain;
}
void PDController::computeControllerInput(){
    position = mRobot -> getGeneralizedCoordinate();
    velocity = mRobot -> getGeneralizedVelocity();
    for(int i = 1; i<4 ; i ++)
    {
        torque[i] = Kp * (desiredPosition[i] - position[i]) + Kd * (desiredVelocity[i] - velocity[i]);

        if (torque[i] >30){
            torque[i] = 30;
        }
        else if(torque[i] <-30){
            torque[i] = -30;
        }
    }
}
