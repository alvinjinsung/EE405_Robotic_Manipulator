/**
 * @file manipulator_control.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr), Jinyeong Jeong (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <chrono>
#include <Eigen/Dense>
#include <csignal>

#include "Controller/ArticulatedSystem.hpp"
#include "FileIO/MatrixFileIO.hpp"
#include "KeyInput/getche.h"

using namespace Eigen;
using namespace std;

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////


int main(int argc, char *argv[])
{       

    // The manipulator has 4 DOF
    int dof = 4;

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
                                        dynamixel_id_set, position_resolution, motor_torque_constants);
    
    Eigen::VectorXd waypoint1(dof); waypoint1.setZero(); // the first user-defined waypoint for direct teaching
    Eigen::VectorXd waypoint2(dof); waypoint2.setZero(); //  the second user-defined waypoint for direct teaching
    char key; // the keyboard input


    std::vector<Eigen::VectorXd> stack_UserDefinedWaypoints;  // We will save two user-defined waypoints in csv files

    articulated_system.DisableTorque();
    while(1){ 
        cout << "To save (i)-th way point, press i (i=1 or 2). To end this process, press 'e'" << endl;
        key = getch(); // Get keyboard input from the user

        if(key == '1') { // The keyboard input is 1
            waypoint1 =  articulated_system.GetJointAngle();
            cout << "Save the first user-defined waypoint" << endl;
            cout << "waypoint1 :" <<  endl;
            cout << waypoint1 << endl;

        }
        else if(key=='2'){ //  The keyboard input is 2
            waypoint2 =  articulated_system.GetJointAngle();
            cout << "Save the second user-defined waypoint" << endl;
            cout << "waypoint2 :" <<  endl;
            cout << waypoint2 << endl;
        }
        else if(key=='e') break;
        else{ // wrong keyboard input
            cout << "Wrong keyboard input" << endl;
        }
    }
    stack_UserDefinedWaypoints.emplace_back(waypoint1); // stack waypoint1
    stack_UserDefinedWaypoints.emplace_back(waypoint2); // stack waypoint2


    // Save the two waypoints to a CSV file named "waypoints.csv" in the same directory as the executable file.
    MatrixXd waypoints_matrix = MatrixFileIO::ConvertStdEigenVectorToEigenMatrix(stack_UserDefinedWaypoints);
    MatrixFileIO::saveData("waypoints.csv", waypoints_matrix);
    

    return 0;
}
