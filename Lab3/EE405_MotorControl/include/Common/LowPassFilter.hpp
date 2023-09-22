/**
 * @file LowPassFilter.hpp
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

class LowPassFilter
{
private:
    // Eigen::MatrixXd u;
    Eigen::VectorXd prev_u;
    Eigen::VectorXd y;
    Eigen::VectorXd prev_y;

    double L; // cut off frequency
    double T; // samplig period. (=dt)
    
public:
    // LowPassFilter(double L_args, double T_args, size_t matrix_rows_args, size_t matrix_cols_args);
    LowPassFilter(const Eigen::VectorXd& init_matrix, double L_args, double T_args);
    // ~LowPassFilter();

    void InitializeMemberValue(const Eigen::VectorXd& prev_u_args, 
                               const Eigen::VectorXd& y_args, 
                               const Eigen::VectorXd& prev_y_args );

    Eigen::VectorXd FilterAndGetY(const Eigen::VectorXd& u);
    
    
};
