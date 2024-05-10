// You can't include a library like this (not local folders!)
#include "admittance_model.hpp"


// Constructor
Controller::Controller(const std::vector<double>& mass,const std::vector<double>& damping)
{

    //Initialize our variables in the constructor
    Mass <<mass[0],0,0,0,0,0,
        0,mass[1],0,0,0,0,
        0,0,mass[2],0,0,0,
        0,0,0,mass[3],0,0,
        0,0,0,0,mass[4],0,
        0,0,0,0,0,mass[5];

    Damping <<damping[0],0,0,0,0,0,
            0,damping[1],0,0,0,0,
            0,0,damping[2],0,0,0,
            0,0,0,damping[3],0,0,
            0,0,0,0,damping[4],0,
            0,0,0,0,0,damping[5];

    // Stiffness.setZero();
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
    Eigen::Matrix<double, 6, 1> DeltaV_desired = (Mass.fullPivLu().solve(Wrench - Damping * Vel)) * Dt;
    
    //Update current Vel
    Eigen::Matrix<double, 6, 1> Vel_desired = Vel + DeltaV_desired;
    setVel(Vel_desired);

    //Return Vel_desired
    return Vel_desired;
}

//Getters
Eigen::Matrix<double, 6, 6>& Controller::getMass() {
    return Mass;   
}
Eigen::Matrix<double, 6, 6>& Controller::getDamping() {
    return Damping;
}
// Eigen::Matrix<double, 6, 6>& Controller::getStiffness() {
//     return Stiffness;
// }
Eigen::Matrix<double, 6, 1>& Controller::getWrench() {
    return Wrench;
}
Eigen::Matrix<double, 6, 1>& Controller::getVel(){
    return Vel;
}

//Setters
void Controller::setMass(Eigen::Matrix<double, 6, 6>& mass){
    Mass=mass;
}
void Controller::setDamping(Eigen::Matrix<double, 6, 6>& damping){
    Damping=damping;
}
// void Controller::setStiffness(Eigen::Matrix<double, 6, 6>& stiffness){
//     Stiffness=stiffness;
// }
void Controller::setWrench(Eigen::Matrix<double, 6, 1>& wrench){
    Wrench=wrench;
}
void Controller::setVel(Eigen::Matrix<double, 6, 1>& vel){
    Vel=vel;
}


