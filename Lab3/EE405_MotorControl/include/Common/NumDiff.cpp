/**
 * @file NumDiff.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "NumDiff.hpp"

NumDiff::NumDiff(const Eigen::VectorXd& u, double dt_args)
: dt(dt_args)
{
    InitializeMemberValue(u);
}

void NumDiff::InitializeMemberValue(const Eigen::VectorXd& prev_u_args)
{
    prev_u = prev_u_args;
}

Eigen::VectorXd NumDiff::ComputeNumericalDerivative(const Eigen::VectorXd& u)
{
    Eigen::VectorXd y = (u-prev_u)/dt; /*This answer is incorrect, Remove y=u, Please implement the proper output formula. */
    prev_u = u;
    return y;
}

