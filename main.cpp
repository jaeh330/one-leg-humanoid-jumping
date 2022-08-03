#include <iostream>
#include "raisim/RaisimServer.hpp"
#include <matplotlibcpp.h>
#include <vector>

int main()
{
    sleep(5);
    //raisim world setting
    raisim::World world;
    world.setTimeStep(0.005);
    world.addGround();

    // robot
    auto jineeLeg = world.addArticulatedSystem("\\home\\jaehyeong\\raisimLib\\rsc\\jinee\\jinee_single_leg.urdf");

    raisim::RaisimServer server(&world);
    server.launchServer();
    sleep(5);

    //initialize
    int iteration = 0;
    double deg2rad = 3.141592 / 180.0;
    double rad2deg = 180.0/3.141592;
    Eigen::VectorXd initialJointPosition(jineeLeg->getGeneralizedCoordinateDim());
    std::cout<<jineeLeg->getGeneralizedCoordinateDim()<<std::endl;
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.3;
    initialJointPosition[1] = -60*deg2rad;
    initialJointPosition[2] = 120*deg2rad;
    initialJointPosition[3] = -150*deg2rad;
    jineeLeg->setGeneralizedCoordinate(initialJointPosition);
    sleep(3);
    Eigen::VectorXd torque(jineeLeg -> getDOF());
    raisim::VecDyn position = raisim::VecDyn(4);
    raisim::VecDyn velocity = raisim::VecDyn(4);
    Eigen::VectorXd desiredPosition = Eigen::VectorXd(4);
    Eigen::VectorXd desiredVelocity = Eigen::VectorXd(4);

    //desired position

    desiredPosition << 0.0, -100*deg2rad, 150*deg2rad, -140*deg2rad;
    desiredVelocity.setZero();
    double Kp=250;
    double Kd = 30;
    std::vector<double> JumpingHighVector;
    std::vector<double> torque1Vector;
    std::vector<double> torque2Vector;
    std::vector<double> torque3Vector;
    std::vector<double> velocity1Vector;
    std::vector<double> velocity2Vector;
    std::vector<double> velocity3Vector;
    std::vector<double> position1Vector;
    std::vector<double> position2Vector;
    std::vector<double> position3Vector;
    double timestep=0.005;
    double JumpHigh=0;
    double L1 = 0.16;
    double L2= 0.275;
    double L3= 0.11;

    int state=0;
    int i=0;

    //Force trajectory
    int T = 50;
    int Fmax = 40;


    while(true)
    {
        iteration++;
        //Fz
        double Fz = Fmax - (iteration-3*T/4)^2;
        position = jineeLeg -> getGeneralizedCoordinate();
        torque[1] = (-L1*sin(position[1]) - L2*sin(position[1]+position[2]) - L3*sin(position[1]+position[2]+position[3]))*Fz;
        torque[2] = (-L2*sin(position[1]+position[2]) - L3*sin(position[1]+position[2]))*Fz;
        torque[3] = (-L3*sin(position[1]+position[2]+position[3]))*Fz;






        //PDControl
//        position = jineeLeg -> getGeneralizedCoordinate();
//        velocity = jineeLeg -> getGeneralizedVelocity();
//        for(int i = 1; i<4 ; i ++)
//        {
            torque[i] = Kp * (desiredPosition[i] - position[i]) + Kd * (desiredVelocity[i] - velocity[i]);
            if (torque[i] >30){
                torque[i] = 30;
            }
            else if(torque[i] <-30){
                torque[i] = -30;
            }
//        }
//
//
//        jineeLeg ->setGeneralizedForce(torque);




        //for plot
//        JumpHigh = position[0];
//        torque1Vector.push_back(torque[1]);
//        torque2Vector.push_back(torque[2]);
//        torque3Vector.push_back(torque[3]);
//        velocity1Vector.push_back(velocity[1]);
//        velocity2Vector.push_back(velocity[2]);
//        velocity3Vector.push_back(velocity[3]);
//        position1Vector.push_back(position[1]*rad2deg);
//        position2Vector.push_back(position[2]*rad2deg);
//        position3Vector.push_back(position[3]*rad2deg);
//        JumpingHighVector.push_back(JumpHigh);
//        SimulationTime.push_back((iteration*timestep));









        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        world.integrate();
        if(iteration == 1000) {break;}
    }





//  plot
    std::vector<double> desiredPositionVector1(SimulationTime.size(),desiredPosition[1]*rad2deg);
    std::vector<double> desiredPositionVector2(SimulationTime.size(),desiredPosition[2]*rad2deg);
    std::vector<double> desiredPositionVector3(SimulationTime.size(),desiredPosition[3]*rad2deg);


// plot
    matplotlibcpp::figure(1);
    matplotlibcpp::named_plot("torque1",SimulationTime,torque1Vector);
    matplotlibcpp::named_plot("torque2",SimulationTime,torque2Vector);
    matplotlibcpp::named_plot("torque3",SimulationTime,torque3Vector);
    matplotlibcpp::legend();
    matplotlibcpp::figure(2);
    matplotlibcpp::named_plot("velocity1",SimulationTime,velocity1Vector);
    matplotlibcpp::named_plot("velocity2",SimulationTime,velocity2Vector);
    matplotlibcpp::named_plot("velocity3",SimulationTime,velocity3Vector);
    matplotlibcpp::legend();
    matplotlibcpp::figure(3);
    matplotlibcpp::plot(SimulationTime, desiredPositionVector1, "r--");
    matplotlibcpp::plot(SimulationTime, desiredPositionVector2, "r--");
    matplotlibcpp::plot(SimulationTime, desiredPositionVector3, "r--");
    matplotlibcpp::legend();
    matplotlibcpp::named_plot("position1",SimulationTime,position1Vector);
    matplotlibcpp::named_plot("position2",SimulationTime,position2Vector);
    matplotlibcpp::named_plot("position3",SimulationTime,position3Vector);
    matplotlibcpp::legend();
    matplotlibcpp::figure(4);
    matplotlibcpp::named_plot("Jumping high",SimulationTime,JumpingHighVector);
    matplotlibcpp::show();
}