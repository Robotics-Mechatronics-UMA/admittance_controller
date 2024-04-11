// You can't include a library like this (not local folders!)
#include "/admittance_controller/include/AdmittanceClass.hpp"



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

//Force sensor callback
void Controller::ForceSensorCallback(const Eigen::Matrix<double, 6,1>& wrench_msg) {
    // Next update using Twist
    // Wrench << wrench_msg.linear.x,wrench_msg.linear.y,wrench_msg.linear.z,
    //         wrench_msg.angular.x,wrench_msg.angular.y,wrench_msg.angular.z;

    //Update wrench matrix value
    Wrench = wrench_msg; 

    // Calculate velocity desired using an admittancec controller
    Eigen::Matrix<double, 6, 1> Vel_pub = AdmittanceController();
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
};

Eigen::Matrix<double, 6, 6>& Controller::getDamping() {
    return Damping;
};

Eigen::Matrix<double, 6, 6>& Controller::getStiffness() {
    return Stiffness;
};

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


// Main
int main() {

    // Create object controller
    Controller controller;


    return 0;
}

