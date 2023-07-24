#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Dense>
#include "tf2/utils.h"
#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace casadi;

class Mpc
{
private:
    //mpc params
    int N_;  //horizon
    double dt_;  //step
    //constrains
    double u_max_, u_min_;
    double w_max_, w_min_;
    
    //weights
    DM Q_, R_;

    
    MX X;
    MX U;
    
    Function kinematic_equation_;
    //OptiSol solution_; 报错没有默认构造函数
    unique_ptr<casadi::OptiSol> solution_;
    

public:
    Mpc();
    ~Mpc();

    Function setKinematicEquation();
    void setWeights(vector<double> weights);
    bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states);
    vector<double> getFirstU();
    vector<double> getPredictX();
};

#endif