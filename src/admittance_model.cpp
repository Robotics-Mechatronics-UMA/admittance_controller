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

    Stiffness.setZero();
    Wrench.setZero();
    Vel.setZero();
    Pos.setZero();
    Dt = 0.01;

}

Controller::~Controller()
{
}

// Admittance control method function
Eigen::Matrix<double, 6, 1> Controller::AdmittanceController() {

    //Admittance algorithm
    Eigen::Matrix<double, 6, 1> Delta_Vel = (Mass.inverse()*(Wrench - Damping * Vel-Stiffness*Pos)) * Dt;

    //Update current Vel
    Eigen::Matrix<double, 6, 1> Vel_desired=Vel+Delta_Vel;
    setVel(Vel_desired);

    //Update current Pos
    Eigen::Matrix<double, 6, 1> Delta_Pos=Vel*Dt;
    Eigen::Matrix<double, 6, 1> Pos_desired = Pos + Delta_Pos;
    setPos(Pos_desired);

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
Eigen::Matrix<double, 6, 6>& Controller::getStiffness() {
    return Stiffness;
}
Eigen::Matrix<double, 6, 1>& Controller::getWrench() {
    return Wrench;
}
Eigen::Matrix<double, 6, 1>& Controller::getVel(){
    return Vel;
}
Eigen::Matrix<double, 6, 1>& Controller::getPos(){
    return Pos;
}
double& Controller::getVel_max(){
    return Vel_max;
}
double& Controller::getDt(){
    return Dt;
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
void Controller::setVel(Eigen::Matrix<double, 6, 1>& vel){
    Vel=vel;
}
void Controller::setPos(Eigen::Matrix<double, 6, 1>& pos){
    Pos=pos;
}
void Controller::setVel_max(double& vel_max){
    Vel_max=vel_max;
}
void Controller::setDt(double& dt){
    Dt=dt;
}
