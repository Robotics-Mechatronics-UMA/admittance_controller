#pragma once
#include <eigen3/Eigen/Dense>

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
};
