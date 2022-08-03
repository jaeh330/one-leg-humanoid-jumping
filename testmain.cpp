#include <iostream>
#include "raisim/RaisimServer.hpp"
#include "include/PDController.h"
#include "include/ParameterPlot.h"
#include "include/InitializeJinee.h"
#include "include/computeForceTrajectory.h"


int main()
{
    sleep(5);
    raisim::World world;
    double TimeStep= 0.005;
    world.setTimeStep(TimeStep);
    world.addGround();
    auto jineeLeg = world.addArticulatedSystem("\\home\\jaehyeong\\raisimLib\\rsc\\jinee\\jinee_single_leg.urdf");
    raisim::RaisimServer server(&world);
    server.launchServer();
    sleep(5);


    //initialize

    InitializeJinee Init(jineeLeg, &world);
    Init.InitJineeJointPosition(0.6,-80, 150, -160);
    sleep(3);


    PDController PD(jineeLeg, &world);
    PD.setPDGain(60, 6);
    PD.setDesiredPosition(-50, 80, -120);
//    PD.setDesiredPosition(-110, 160, -140);
//    PD.setDesiredPosition(-80, 120, -130);
//    PD.setDesiredPosition(-60, 100, -130);
    PD.setDesiredVelocity(0,0,0);

    ParameterPlot Plot;

    //Force trajectory  -
    double T = 110;
    double Fmax = 150;
    double Te = 90;
    double Ti = 67;
    double L1 = 0.16;
    double L2= 0.275;
    double L3= 0.11;
    int StartIteration = 300;
    int TrajectoryIteration = 0;
    std::vector<double> ForceplotIteration;
    computeForceTrajectory Force;
    Force.FindCoefficient(T,Ti,Te,Fmax);


    //For plot
    std::vector<double> Fplot;


    while(true)
    {
        Init.iteration++;
        if(Init.iteration < StartIteration){
            PD.computeControllerInput();
        }

        if(Init.iteration >= StartIteration && Init.iteration < StartIteration+T ){
//            PD.position = jineeLeg -> getGeneralizedCoordinate();
//            Force.exponecialF((Init.iteration-StartIteration),T,Te,Fmax);
            Force.computeF(TrajectoryIteration);
            PD.torque[1] = (-L1*sin(PD.position[1]) - L2*sin(PD.position[1]+PD.position[2]) - L3*sin(PD.position[1]+PD.position[2]+PD.position[3]))*Force.F;
            PD.torque[2] = (-L2*sin(PD.position[1]+PD.position[2]) - L3*sin(PD.position[1]+PD.position[2]))*Force.F;
//            PD.torque[2] = PD.Kp * (PD.desiredPosition[2] - PD.position[2]) + PD.Kd * (PD.desiredVelocity[2] - PD.velocity[2]);
            PD.torque[3] = (-L3*sin(PD.position[1]+PD.position[2]+PD.position[3]))*Force.F;

            for(int i = 1; i<4 ; i ++)
            {
                if (PD.torque[i] >30){
                    PD.torque[i] = 30;
                }
                else if(PD.torque[i] <-30){
                    PD.torque[i] = -30;
                }
            }
            TrajectoryIteration++;
            ForceplotIteration.push_back(TrajectoryIteration*0.005);
            Force.F = PD.torque[3]/(-L3*sin(PD.position[1]+PD.position[2]+PD.position[3]));
            Fplot.push_back(Force.F);
            PD.position = jineeLeg -> getGeneralizedCoordinate();
            PD.setDesiredPosition(PD.position[1], PD.position[2], -PD.position[3]);
        }
        if(Init.iteration>StartIteration+T){
            PD.setDesiredPosition(-60, 100, -130);
            PD.computeControllerInput();
//            TrajectoryIteration++;
//            ForceplotIteration.push_back(TrajectoryIteration);
//            Force.F = PD.torque[3]/(-L3*sin(PD.position[1]+PD.position[2]+PD.position[3]));
//            Fplot.push_back(Force.F);
        }

        jineeLeg ->setGeneralizedForce(PD.torque);
        if(Init.iteration == 1000) {break;}
        usleep(10000);
        world.integrate();

        // for plot
        Plot.pushBack(PD.position,PD.velocity,PD.torque, Init.iteration);


    }

    //Plot
    Plot.torquePlot(1);
    matplotlibcpp::figure(2);
    matplotlibcpp::title("Ground Reaction Force in Jumping Procedure");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::ylabel("[N]");
    matplotlibcpp::plot(ForceplotIteration,Fplot);
    matplotlibcpp::legend();
    matplotlibcpp::show();

}