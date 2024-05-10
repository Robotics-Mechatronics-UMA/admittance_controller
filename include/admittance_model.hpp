#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>


class Controller {
private:

    //Variables
    Eigen::Matrix<double, 6, 6> Mass;
    Eigen::Matrix<double, 6, 6> Damping;
    Eigen::Matrix<double, 6, 6> Stiffness;
    Eigen::Matrix<double, 6, 1> Vel;
    Eigen::Matrix<double, 6, 1> Wrench;
    double Dt;


public:

    //Constructor
    Controller(const std::vector<double>& mass,const std::vector<double>& damping);
    //Destructor
    ~Controller();
    //Admittance controller
    Eigen::Matrix<double, 6, 1> AdmittanceController();

    //Get params
    Eigen::Matrix<double, 6, 6>& getMass();
    Eigen::Matrix<double, 6, 6>& getDamping();
    Eigen::Matrix<double, 6, 6>& getStiffness();
    Eigen::Matrix<double, 6, 1>& getWrench();
    Eigen::Matrix<double, 6, 1>& getVel();

    //Set params
    void setMass(Eigen::Matrix<double, 6, 6>&);
    void setDamping(Eigen::Matrix<double, 6, 6>&);
    void setStiffness(Eigen::Matrix<double, 6, 6>&);
    void setWrench(Eigen::Matrix<double, 6, 1>&);
    void setVel(Eigen::Matrix<double, 6, 1>&);
};
