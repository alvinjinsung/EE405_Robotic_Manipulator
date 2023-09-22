/**
 * @file modified_direct_teaching.cpp
 * @author Seo Wook Han (tjdnr7117@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-05-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <chrono>
#include <Eigen/Dense>

#include "Controller/ArticulatedSystem.hpp"
#include "Common/DirectTeaching.hpp"

using namespace Eigen;
using namespace std;

#define SERVERPORT "4950"   // the port users will be connecting to

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////

int main(int argc, char *argv[])
{       
    // The manipulator has 4 DOF
    int dof = 4;

    // The control frequency is 100Hz
    double control_freq = 100;
    double dt = 1/control_freq;

    // Dynamixel setup, ID, position resolution, motor torque constant, etc.
    std::vector<uint8_t> dynamixel_id_set;
    std::vector<int32_t> position_resolution;
    std::vector<double>  motor_torque_constants;
    for (size_t i = 0; i < dof; i++)
    {
        dynamixel_id_set.emplace_back(i); 
        position_resolution.emplace_back(POSITION_RESOLUTION);
        motor_torque_constants.emplace_back(MOTOR_TORQUE_CONSTANT);
    }
    
    // Initialize the Articulated_system, 
    // We need a dof of the system, Operation Mode(Position, Velocity, Current),
    // USB Port information (for serical communication),
    // and dynamixel setup
    ArticulatedSystem articulated_system(dof, ArticulatedSystem::Mode::VELOCITY, "/dev/ttyUSB0", 
                                        dynamixel_id_set, position_resolution, motor_torque_constants); //4 DOF manipulator

    /**
     * The member function articulated_system.GetJointAngle() returns the joint angles of the robot system, which in this context refer to the motor angle.
     */
    Eigen::VectorXd currentQ = articulated_system.GetJointAngle();
    Eigen::VectorXd initialQ = articulated_system.GetJointAngle();

    // Declare targetQ(motor angle), targetQdot(motor velocity)
    Eigen::VectorXd targetQ(dof); targetQ.setZero();
    Eigen::VectorXd targetQdot(dof); targetQdot.setZero();

    /**
     * @brief time instant is initialized.
     * loop_start : represents the moment when the 'while' loop starts.
     * loop_end : represents the moment when the 'while' loop ends.
     * current_time : current time instant. 
     * initial_time : The time instant when the program(=main function) starts.
     */
    std::chrono::system_clock::time_point loop_start= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point loop_end= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point current_time= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point initial_time= std::chrono::system_clock::now();
    
    // P gain.
    Eigen::MatrixXd Kp(dof, dof); Kp.setZero();

    //for direct teaching
    DirectTeaching direct_teaching(dof);
    Eigen::Matrix<double,2,4> task1_waypoints; //waypoints for task1, row 0 : start position, row 1 : target position
    Eigen::Matrix<double,2,4> task2_waypoints; //waypoints for task2, row 0 : start position, row 1 : target position
    direct_teaching.initialization(initialQ);
    task1_waypoints.row(0) = initialQ; /* Implement here. Initialize the start point of task1_waypoints*/
    task2_waypoints.row(0) = initialQ; /* Implement here. Initialize the start point of task2_waypoints*/
    
    Eigen::MatrixXd stack_waypoints = direct_teaching.get_stack_waypoints();
    task1_waypoints.row(1) = stack_waypoints.row(1); /* Implement here. Set the final point of task1_waypoints as first row of 'waypoints.csv' */
    task2_waypoints.row(1) = stack_waypoints.row(2); /* Implement here. Set the final point of task1_waypoints as second row of 'waypoints.csv'*/

    int mode = 0; // 0 : tracking task1_waypoints, 1 : tracking task2_waypoints
    
    while(1)
    {
        current_time= std::chrono::system_clock::now();
        auto loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - loop_start);
        auto total_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - initial_time);

        // For 100 Hz control loop.
        if(loop_elasped_time_microsec.count()>=dt*1e6)
        {
            double total_elasped_time_sec = static_cast<double>(total_elasped_time_microsec.count())/1e6;
            loop_start = std::chrono::system_clock::now();

            currentQ = articulated_system.GetJointAngle();

            // ///////////////////////////////////// Direct teaching mode /////////////////////////////////////
            if(mode == 0)
            {
                // trajectory generation from task1_waypoints_.row(0)(current position) to task1_waypoints_.row(1)
                targetQ = direct_teaching.TrajectoryGeneration(task1_waypoints);
                int num_task_waypoints = task1_waypoints.rows();
                
                //define transition event (task1-> task2) : norm of error in position is less than certain threshold
                const double threshold = 0.01;
                Eigen::VectorXd error = currentQ - task1_waypoints.row(1).transpose(); /* Implement here. error is defined as ‘currentQ - final point of task1_waypoints*/

                // mode transition event
                if(error.norm() < threshold /* Implement here. codition of mode transition */)
                {   
                    std::cout<<"mode change"<<std::endl;
                    mode = 1;
                    direct_teaching.clear();
                    task2_waypoints.row(0) = currentQ; /* Implement here. update start point of task2_waypoints */
                }
            }
            else if(mode == 1)
            {
                // trajectory generation from task1_waypoints_.row(0)(current position) to task1_waypoints_.row(1)
                targetQ = direct_teaching.TrajectoryGeneration(task2_waypoints);
            }
            // ///////////////////////////////////// Direct teaching mode END /////////////////////////////////////

            ///////////////////////////////////// P Position control USING Velocity Mode /////////////////////////////////////
            for (size_t i = 0; i < dof; i++)
            {
                Kp(i,i) = 4;
            }           
            targetQdot = -Kp * (currentQ - targetQ);
            articulated_system.SetGoalVelocity(targetQdot); // velocity control mode
            ///////////////////////////////////// P Position control USING Velocity Mode END /////////////////////////////////////

            loop_end= std::chrono::system_clock::now();
            auto one_step_calculation_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
        }

    }

    return 0;
}
