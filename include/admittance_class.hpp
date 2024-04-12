#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>

class Controller {
private:

    //Variables
    Eigen::Matrix<double, 6, 6> Mass;
    Eigen::Matrix<double, 6, 6> Damping;
    Eigen::Matrix<double, 6, 6> Stiffness;
    Eigen::Matrix<double, 6, 1> Vel;
    Eigen::VectorXd Wrench;
    double Dt;


public:
    //Constructor
    Controller();
    //Destructor
    ~Controller();
    //Admittance controller
    Eigen::Matrix<double, 6, 1> AdmittanceController();

    //Get params
    Eigen::Matrix<double, 6, 6>& Controller::getMass();
    Eigen::Matrix<double, 6, 6>& getDamping();
    Eigen::Matrix<double, 6, 6>& Controller::getStiffness();
    //Set params
    void Controller::setMass(Eigen::Matrix<double, 6, 6>&);
    void Controller::setDamping(Eigen::Matrix<double, 6, 6>&);
    void Controller::setStiffness(Eigen::Matrix<double, 6, 6>&);

};
