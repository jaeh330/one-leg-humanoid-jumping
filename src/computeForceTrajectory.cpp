//
// Created by jaehyeong on 22. 6. 9.
//
#include "../include/computeForceTrajectory.h"

void computeForceTrajectory::FindCoefficient(double T, double Ti, double Te, double Fmax) {
    Eigen::Matrix4d A;
    A << T,pow(T,2),pow(T,3),pow(T,4),
         Te,pow(Te,2),pow(Te,3),pow(Te,4),
         1,2*Te,3*pow(Te,2),4*pow(Te,3),
         0,2,6*Ti,12*pow(Ti,2);

    Eigen::Matrix <double, 4,1> B;
    B << -generalForce,
         Fmax-generalForce,
         0,
         0;

    Coefficient = A.inverse()*B;
}

void computeForceTrajectory::computeF(int iteration){
    F = generalForce + Coefficient(0)*iteration + Coefficient(1)*pow(iteration,2) + Coefficient(2)*pow(iteration,3) + Coefficient(3)*pow(iteration,4);
}

void computeForceTrajectory::exponecialF(int iteration, double T, double Te, double Fmax){
    double A = (Fmax-generalForce)/(pow(1.1,Te)-1);
    double B = generalForce - A;
    if( iteration <= Te){
        F = A*pow(1.1,iteration) +B;
    }
    else{
//        F = generalForce + Coefficient(0)*iteration + Coefficient(1)*pow(iteration,2) + Coefficient(2)*pow(iteration,3) + Coefficient(3)*pow(iteration,4);
        double Fmin = -50;
        F = (Fmax-Fmin)/(Te - T)*(iteration - T)+Fmin;
    }


}

