/**
 * @file motor_control.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr), Jinyeong Jung (jinyeong.jeong@kaist.ac.kr)
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
#include "Common/LowPassFilter.hpp"
#include "Common/NumDiff.hpp"

using namespace Eigen;
using namespace std;

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////

bool g_isShutdown = false;

/**
 * @brief Pressing Ctrl + C in a terminal typically sends a SIGINT signal that can cause a running program to terminate. 
 *          This thread continously  check the signal to stop main function.
 * 
 * @param signum signal number: if you push the Ctrl + C on the terminal, It has a positive interger number.
 */
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    // Cleanup and close up things here
    // Terminate program
    if(signum > 0)
    {
        g_isShutdown = true;
    }
}

int main(int argc, char *argv[])
{       
    signal(SIGINT, signalHandler); // For Checking the Ctrl + C.

    int dof = 1;

    double control_freq = 200;
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
    ArticulatedSystem articulated_system(dof, ArticulatedSystem::Mode::CURRENT, "/dev/ttyUSB0", 
                                        dynamixel_id_set, position_resolution, motor_torque_constants); //1 DOF manipulator (In other words, one motor system)

    /**
     * The member function articulated_system.GetJointAngle() returns the joint angles of the robot system, which in this context refer to the motor angle.
     * The member function articulated_system.GetJointVelocity() returns the joint velocities of the robot system, which in this context refer to the motor velocity.
     */
    Eigen::VectorXd currentQ = articulated_system.GetJointAngle();
    Eigen::VectorXd currentQdot = articulated_system.GetJointVelocity();

    // numerical differentiation & Low pass filter was intialized. 
    // the second parameter of LowPassFilter is the cutoff frequency.
    NumDiff numDiff_for_currentQdot(currentQ, dt);
    LowPassFilter lpf_qdot(currentQdot, 50, dt);

    // Declare targetQ(motor angle), targetQdot(motor velocity), targetTorque(target motor torque).
    Eigen::VectorXd targetQ(dof); targetQ.setZero();
    Eigen::VectorXd targetQdot(dof); targetQdot.setZero();
    Eigen::VectorXd targetTorque(dof); targetTorque.setZero();

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

    // We will use the matlab to plot data using csv files.
    std::vector<Eigen::VectorXd> save_for_plot;
    
    // PID gain.
    Eigen::MatrixXd Kp(dof, dof); Kp.setZero();
    Eigen::MatrixXd Ki(dof, dof); Ki.setZero();
    Eigen::MatrixXd Kd(dof, dof); Kd.setZero();

    // the integration of the error
    Eigen::VectorXd integral_error(dof); integral_error.setZero();

    // Please refer to the signalHandler function. If g_isShutdown is true, the program will be shut down.
    while(!g_isShutdown){
        current_time= std::chrono::system_clock::now();
        auto loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - loop_start);
        auto total_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - initial_time);

        // For 200 Hz control loop.
        if(loop_elasped_time_microsec.count()>=dt*1e6)
        {
            double total_elasped_time_sec = static_cast<double>(total_elasped_time_microsec.count())/1e6;
            loop_start = std::chrono::system_clock::now();

            currentQ = articulated_system.GetJointAngle();
            currentQdot = articulated_system.GetJointVelocity(); // Get the ROBOTIS's motor velocity for comparison.

            /**
             * @brief numerical derivative, and then apply the low pass filter to eliminate the high frequency noise.
             * @attention [Problem 3B] You should implement following functions. 
             *            Just Ctrl + click "ComputeNumericalDerivative" and "FilterAndGetY".
             *            Or please refer to following diretory and code.
             *              
             *              ComputeNumericalDerivative => include/Common/NumDiff.hpp,  include/Common/NumDiff.cpp
             *              FilterAndGetY => include/Common/LowPassFilter.hpp,  include/Common/LowPassFilter.cpp
             */
            Eigen::VectorXd numdiff_currectQdot = numDiff_for_currentQdot.ComputeNumericalDerivative(currentQ);
            Eigen::VectorXd filtered_numdiff_currentQdot = lpf_qdot.FilterAndGetY(numdiff_currectQdot);

            double amp = 1;
            ////////////////// fixed frequency //////////////////            
            // double sin_wave_frequency = 2*M_PI*0.77; // [\omega, rad / s] 
            ////////////////// fixed frequency END //////////////////

            ////////////////// For Chirp Signal //////////////////
            double sin_wave_frequency = 2*M_PI*0.1*total_elasped_time_sec; // [rad / s] for chirp signal.
            ////////////////// For Chirp Signal //////////////////

            ///////////// Constant target /////////////
            // targetQ << 1;// M_PI;
            // targetQdot << 0;
            ///////////// Constant target END /////////////

            ///////////// Sine wave target /////////////
            targetQ << amp*sin(sin_wave_frequency*total_elasped_time_sec);
            targetQdot << 2 * sin_wave_frequency*amp*cos(sin_wave_frequency*total_elasped_time_sec);
            ///////////// Sine wave target END /////////////

            ///////////////// POSITION CONTROL //////////////////////
            // articulated_system.SetGoalPosition(targetQ);
            ///////////////// POSITION CONTROL END //////////////////////

            ///////////////// VELOCITY CONTROL //////////////////////
            // articulated_system.SetGoalVelocity(targetQdot);
            ///////////////// VELOCITY CONTROL END //////////////////////
            
            ///////////////// CURRENT CONTROL //////////////////////
            // targetTorque.setZero();
            // articulated_system.SetGoalTorque(targetTorque);
            ///////////////// CURRENT CONTROL END //////////////////////
            
            ////////////// Velocity P control USING Current Mode //////////////////////
            // for (size_t i = 0; i < dof; i++)
            // {
            //     Kp(i,i) = 0.01;
            // }           
            // targetTorque = -Kp * (currentQdot - targetQdot) ;
            // articulated_system.SetGoalTorque(targetTorque);
            ////////////// Velocity P control USING Current Mode END //////////////////////

            ///////////////// CURRENT CONTROL //////////////////////
            // ////////// PID Position controller //////////////////////
            for (size_t i = 0; i < dof; i++)
            {
                Kp(i,i) = 0.5;
                Ki(i,i) = 0.1;
                Kd(i,i) = 0.05;
            }
            Eigen::VectorXd error = currentQ - targetQ;
            Eigen::VectorXd error_dot = currentQdot - targetQdot;
            integral_error += error *dt;
            targetTorque = -Kp*error -Kd*error_dot -Ki*integral_error;/* Implement here. you have to implement the PID Controller */
            articulated_system.SetGoalTorque(targetTorque);
            ////////////// PID Position controller END //////////////////////
            ///////////////// CURRENT CONTROL END ////////////////////// 
            

            loop_end= std::chrono::system_clock::now();
            auto one_step_calculation_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
            
            /**
             * @brief stack all the relevant data for each time step, for the purpose of plotting.
             */
            Eigen::VectorXd save_for_plot_data(1 + // total_elasped_time (scalar)
                                               1 + // loop_elasped_time(scalar) 
                                               1 + // one_step_calculation_time (scalar)
                                               currentQ.size() + 
                                               currentQdot.size() + 
                                               targetQ.size() + 
                                               targetQdot.size() + 
                                               targetTorque.size() + 
                                               numdiff_currectQdot.size() +
                                               filtered_numdiff_currentQdot.size());

            save_for_plot_data << static_cast<double>(total_elasped_time_microsec.count())/1e6,
                                  static_cast<double>(loop_elasped_time_microsec.count())/1e6,
                                  static_cast<double>(one_step_calculation_time.count())/1e6,
                                  currentQ,
                                  currentQdot,
                                  targetQ,
                                  targetQdot,
                                  targetTorque,
                                  numdiff_currectQdot,
                                  filtered_numdiff_currentQdot;
            save_for_plot.emplace_back(save_for_plot_data);
        

        }      
    }

    // Save the recorded data matrix to a CSV file named "recorded_data.csv" in the same directory 
    // as the executable file.
    MatrixXd save_for_plot_data_matrix = MatrixFileIO::ConvertStdEigenVectorToEigenMatrix(save_for_plot);
    MatrixFileIO::saveData("recorded_data.csv", save_for_plot_data_matrix);
    

    return 0;
}
