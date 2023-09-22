/**
 * @file DirectTeaching.cpp
 * @author Jinyeong Jeong (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <Eigen/Dense>
#include "FileIO/MatrixFileIO.hpp"
#include "KeyInput/getche.h"
#include <fstream>
#include <string>

class DirectTeaching
{
private:
    Eigen::MatrixXd stack_waypoints;
    int NumWaypoints;
    int dof;
    int NumNodes;
    int NodeIndex;
    int StartPointIndex;
    int EndPointIndex;

public:
    DirectTeaching(int dof_args);
    void initialization(const Eigen::VectorXd &current_q);
    void clear();
    Eigen::VectorXd TrajectoryGeneration(const Eigen::MatrixXd & task_waypoints);
    Eigen::MatrixXd get_stack_waypoints();
};

