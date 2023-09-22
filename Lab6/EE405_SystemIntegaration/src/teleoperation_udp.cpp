/**
 * @file teleoperation_udp.cpp
 * @author Seo Wook Han (tjdnr7117@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-05-01
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
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/time.h>

#include "Controller/ArticulatedSystem.hpp"
#include "Common/Kinematics.hpp"
#include "Common/gpio_control.h"

using namespace Eigen;
using namespace std;

#define SERVERPORT "4950"   // the port users will be connecting to

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////

// GetKeyboardInput() function : Compute the task command from the keyboard input for teleopeartion.
// Keyboard input should be received from the local PC shell.
// You can copy and paste the code that is implemented in lab4 (RemoteController_PC_Skeleton.cpp)

bool is_key_updated = false; 
Eigen::Vector3d task_command;
char key='i';
void GetKeyboardInput()
{   
    // initialize variables and error print
    // Get argument of destination IP (argv) of Bone
    // Init datagram socket. You will use UDP network.
    
    /*Implement here. Initialize all the variables required for udp communication*/
    char buf;

    int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv;
	int numbytes;
	struct sockaddr_storage their_addr;
	socklen_t addr_len;
	char s[INET6_ADDRSTRLEN];

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET6; // set to AF_INET to use IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	if ((rv = getaddrinfo(NULL, SERVERPORT, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return;
	}

	// loop through all the results and bind to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("listener: socket");
			continue;
		}

		if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("listener: bind");
			continue;
		}

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "listener: failed to bind socket\n");
		return;
	}

    while(1)
    {   
        printf("+-----------------------------------------------------+\n");
        printf("|   q: GPIO |  w: +y   |  e: stop |  r: +z  | i: init |\n");
        printf("|-----------+----------+----------+-------------------|\n");
        printf("|   a: -x   |  s: -y   |  d: +x   |  f: -z  |         |\n");
        printf("+-----------------------------------------------------+\n");

        /*Implement here. "Fill the 'buf' with the data received through UDP*/

        addr_len = sizeof their_addr;
        if ((numbytes = recvfrom(sockfd, &buf, sizeof(buf), 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
            perror("recvfrom");
            exit(1);
        }

        std::cout<<"listener: packet contains "<<buf<<std::endl;
        
        key = buf;
        is_key_updated=true; // When a new keyboard input is received, is_key_upated is set to true
    }

    /*Implement here. close the socket*/

    freeaddrinfo(servinfo);
	close(sockfd);
}
int main(int argc, char *argv[])
{       
    std::thread t1 = std::thread(GetKeyboardInput); // Create thread t1 to get the keyboard input from the user

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

    bool is_ready_for_teleoperation=false;
    Eigen::VectorXd prev_targetQ(dof); // Declare prev_targetQ(the previous targetQ value)
    
    // The initial joint position for teleoperation is [0, 0, PI/2, -PI/2]. This configuraiton is far from singularities.
    Eigen::VectorXd initial_position(dof); initial_position << 0, 0, M_PI/2, -M_PI/2; 
    prev_targetQ = initial_position;

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
    
    // The following variables are for trajectory generation before starting teleoperation
    int NodeIndex=0;
    int NodeNum=500;

    //initialize gpio port
    int fd_gpio_30;
    gpio_export(30);
    gpio_set_dir(30, 1);
    fd_gpio_30 = gpio_fd_open(30);
    gpio_fd_set_value(fd_gpio_30, 1);
    bool is_magnetic_on_ = false;
    
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
            currentQ.resize(4);

            if(!is_ready_for_teleoperation){ // Move the end-effector to initial_position before starting teleoperation
                targetQ = (initial_position - initialQ)/NodeNum*NodeIndex+initialQ; // trajectory generation using the first-order polynomial
                NodeIndex++;
                if(NodeIndex>NodeNum) {
                    is_ready_for_teleoperation=true; // Now, it is ready for teleopeartion
                    std::cout<<"Start teleoperation!"<<std::endl;
                }
            }

            else{
                ///////////////////////////////////// Teleoperation mode Start /////////////////////////////////////
                if(is_key_updated){ // Perform teleoperation only when a new keyboard input is received (is_key_upated=true)                
                    if(key=='i') // Move to the initial position for teleoperation
                    {
                        targetQ=initial_position; 
                        prev_targetQ=targetQ; // Update prev_targetQ
                    }
                    else if(key=='r' || key=='f' || key=='w' || key=='s' || key=='d' || key=='a') // Move the end-effector based on the task command
                    {   
                        double position_resolution=0.01; // The position resolution for teleoperation is 1cm
                        if(key == 'r') { 
                            task_command<< 0, 0, position_resolution; /*Implement here*/ 
                        } 
                        else if(key == 'f') { 
                            task_command<< 0, 0, -position_resolution; /*Implement here*/  
                        } 
                        else if(key == 'w') { 
                            task_command<< 0, position_resolution, 0; /*Implement here*/ 
                        } 
                        else if(key == 's') { 
                            task_command<< 0, -position_resolution, 0; /*Implement here*/ 
                        } 
                        else if(key == 'd') { 
                            task_command<< position_resolution, 0, 0; /*Implement here*/ 
                        } 
                        else if(key == 'a') { 
                            task_command<< -position_resolution, 0, 0; /*Implement here*/ 
                        }
                        Eigen::MatrixXd J=Kinematics::GetJacobianMatrix(currentQ); // J is the Jacobian matrix for linear velocity represented in the world frame
                        targetQ = currentQ + (J.transpose()*(J*J.transpose()).inverse()) * task_command; /*Implement here*/ 
                        prev_targetQ=targetQ; // Update prev_targetQ
                    }
                    else if(key=='q')
                    {  
                        //When the key input is 'q', if the magnet is currently off, turn it on. If it is currently on, turn it off
                        is_magnetic_on_ = !is_magnetic_on_; /*Implement here. */ 

                        if(is_magnetic_on_ == true)
                        {
                            std::cout<<"Magnetic ON"<<std::endl;
                        }
                        else
                        {
                            std::cout<<"Magnetic OFF"<<std::endl;
                        }

                        gpio_fd_set_value(fd_gpio_30, is_magnetic_on_);

                        targetQ=prev_targetQ;
                    }
                    else if(key=='e'){ // End teleoperation
                        break;
                    }
                    else // When the wrong keyboard input is received, the targetQ is the privious targetQ value
                    {
                        targetQ=prev_targetQ;
                    }

                    is_key_updated=false; // Set is_key_updated to false in order to check if the new keyboard input is received
                }            
                else{ // Before receiving the new keyboard input, the targetQ is the privious targetQ value
                    targetQ=prev_targetQ;
                }        
                ///////////////////////////////////// Teleoperation mode END /////////////////////////////////////
            }

            ///////////////////////////////////// P Position control USING Velocity Mode /////////////////////////////////////
            for (size_t i = 0; i < dof; i++)
            {
                Kp(i,i) = 4;
            }           
            targetQdot = -Kp * (currentQ - targetQ); /* Implement here. You have to implement joint space P position Controller */
            articulated_system.SetGoalVelocity(targetQdot); // velocity control mode
            ///////////////////////////////////// P Position control USING Velocity Mode END /////////////////////////////////////
            loop_end= std::chrono::system_clock::now();
            auto one_step_calculation_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
        }
    }

    t1.detach(); //  allows the main function to wait until t1 completes its execution

    gpio_unexport(30);

    return 0;
}