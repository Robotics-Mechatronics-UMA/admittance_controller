#ifndef ADMITANCE_CONTROLLER_H
#define ADMITANCE_CONTROLLER_H
#include <eigen3/Eigen/Dense>
#include <iostream>

class Controller {
private:

    Eigen::Matrix<double, 6, 6> Mass;
    Eigen::Matrix<double, 6, 6> Damping;
    Eigen::Matrix<double, 6, 6> Stiffness;
    Eigen::Matrix<double, 6, 1> Vel;
    Eigen::VectorXd Wrench;
    double Dt;


public:
    Controller();
    void ForceSensorCallback(const Eigen::Matrix<double, 6,1>&);
    Eigen::Matrix<double, 6, 1> AdmittanceController();

    Eigen::Matrix<double, 6, 6>& Controller::getMass();
    Eigen::Matrix<double, 6, 6>& getDamping();
    Eigen::Matrix<double, 6, 6>& Controller::getStiffness();

    void Controller::setMass(Eigen::Matrix<double, 6, 6>&);
    void Controller::setDamping(Eigen::Matrix<double, 6, 6>&);
    void Controller::setStiffness(Eigen::Matrix<double, 6, 6>&);

};

#endif // ADMITANCE_CONTROLLER_H