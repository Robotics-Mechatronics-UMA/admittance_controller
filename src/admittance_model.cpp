// You can't include a library like this (not local folders!)
#include "include/admittance_class.hpp"


// Constructor
Controller::Controller(){

    //Initialize our variables in the constructor
    Mass <<30,0,0,0,0,0,
        0,30,0,0,0,0,
        0,0,30,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0;

    Damping <<20,0,0,0,0,0,
            0,20,0,0,0,0,
            0,0,20,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0;

    Stiffness.setZero();
    Wrench.setZero();
    Vel.setZero();
    //Integration time
    Dt=0.01;

}

Controller::~Controller()
{
}

// Admittance control method function
Eigen::Matrix<double, 6, 1> Controller::AdmittanceController() {
    Eigen::Matrix<double, 6, 1> Vel_desired = (Mass.fullPivLu().solve(Wrench - Damping * Vel)) * Dt;
    Vel = Vel_desired;
    return Vel_desired;
}

//Getters
Eigen::Matrix<double, 6, 6>& Controller::getMass() {
    return Mass;   
}
Eigen::Matrix<double, 6, 6>& Controller::getDamping() {
    return Damping;
}
Eigen::Matrix<double, 6, 6>& Controller::getStiffness() {
    return Stiffness;
}
Eigen::Matrix<double, 6, 1>& Controller::getWrench() {
    return Wrench;
}


//Setters
void Controller::setMass(Eigen::Matrix<double, 6, 6>& mass){
    Mass=mass;
}
void Controller::setDamping(Eigen::Matrix<double, 6, 6>& damping){
    Damping=damping;
}
void Controller::setStiffness(Eigen::Matrix<double, 6, 6>& stiffness){
    Stiffness=stiffness;
}
void Controller::setWrench(Eigen::Matrix<double, 6, 1>& wrench){
    Wrench=wrench;
}


