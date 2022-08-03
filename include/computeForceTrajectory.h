//
// Created by jaehyeong on 22. 6. 9.
//

#ifndef RAISIM_COMPUTEFORCE_H
#define RAISIM_COMPUTEFORCE_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include "raisim/RaisimServer.hpp"


class computeForceTrajectory {
public:

    void FindCoefficient( double T, double Ti, double Te, double Fmax);
    void computeF(int iteration);
    void exponecialF(int iteration,double T, double Te, double Fmax);
    double F;
    Eigen::Matrix <double, 4,1> Coefficient;
    double hipMass = 0.7835;
    double upperMass = 0.3;
    double lowerMass =0.5;
    double footMass = 0.3;
    double generalForce = (hipMass + upperMass + lowerMass + footMass) * 9.81;

};
#endif //RAISIM_COMPUTEFORCE_H
