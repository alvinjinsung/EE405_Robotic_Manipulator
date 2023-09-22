/**
 * @file LowPassFilter.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "LowPassFilter.hpp"

LowPassFilter::LowPassFilter(const Eigen::VectorXd& init_matrix, double L_args, double T_args) 
: L(L_args), T(T_args)
{
    InitializeMemberValue(init_matrix, init_matrix, init_matrix);
}

void LowPassFilter::InitializeMemberValue(const Eigen::VectorXd& prev_u_args, 
                            const Eigen::VectorXd& y_args, 
                            const Eigen::VectorXd& prev_y_args )
{
    prev_u = prev_u_args;
    y = y_args;
    prev_y = prev_y_args;
}

Eigen::VectorXd LowPassFilter::FilterAndGetY(const Eigen::VectorXd& u)
{
    y = ((2-L*T)/(2+L*T)) * prev_y + ((L*T)/(2+L*T)) *(u+prev_u); /*This answer is incorrect, Remove y=u, Please implement the proper output formula. */
    prev_u = u;
    prev_y = y;
    return y;
}
    
    
