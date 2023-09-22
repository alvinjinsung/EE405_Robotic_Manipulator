/**
 * @file NumDiff.hpp
 * @author Jiwan Han (jw.han@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <Eigen/Dense>

class NumDiff
{
private:
    Eigen::VectorXd prev_u;
    double dt;

public:
    NumDiff(const Eigen::VectorXd& u, double dt_args);

    void InitializeMemberValue(const Eigen::VectorXd& prev_u_args);

    Eigen::VectorXd ComputeNumericalDerivative(const Eigen::VectorXd& u);
};

