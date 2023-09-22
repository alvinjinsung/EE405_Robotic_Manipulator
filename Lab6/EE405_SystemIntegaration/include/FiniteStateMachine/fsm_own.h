#ifndef FSM_H
#define FSM_H

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <fstream>
#include "Common/gpio_control.h"
#include "Common/Kinematics.hpp"
#include "Common/DirectTeaching.hpp"

enum FSM_state
{   
    FSM_IDLE = 0, // maintain current position
    FSM_TASK1 = 1, // tracking task1_waypoints_
    FSM_TASK2 = 2, // task space control mode
    FSM_TASK3 = 3 // tracking task2_waypoints_
};

//Finite state machine
class FSM
{   
    private:
    //common variables
    const int dof_;

    //For Task1,3 (Direct teaching)
    DirectTeaching direct_teaching_;
    Eigen::Matrix<double,2,4> task1_waypoints_; //waypoints for task1, row 0 : start position, row 1 : target position
    Eigen::Matrix<double,2,4> task2_waypoints_; //waypoints for task2, row 0 : start position, row 1 : target position

    //For Task2 (task sapce control)
    Eigen::VectorXd prev_targetQ;
    bool is_magnetic_on_ = false;
    bool is_light_on_ = false;

    public:
    FSM_state state = FSM_IDLE; // set initial state as idel.

    FSM(const int & dof);

    void initialization(const Eigen::VectorXd &current_q);
    void print_state();
    // maintain current position
    Eigen::VectorXd Idle(const char &key_input);
    // tracking saved waypoints
    Eigen::VectorXd Task1(const Eigen::VectorXd &current_q);
    // task space control mode
    Eigen::VectorXd Task2(const char &key_input, const bool& is_key_updated, const Eigen::VectorXd &q, const int &fd_gpio_30, const int &fd_gpio_31);
    // tracking saved waypoints in reverse order
    Eigen::VectorXd Task3(const Eigen::VectorXd &current_q);


};

#endif