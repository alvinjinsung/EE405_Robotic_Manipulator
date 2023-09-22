#include "fsm_own.h"

FSM::FSM(const int & dof) : dof_(dof), direct_teaching_(dof){}

void FSM::initialization(const Eigen::VectorXd &current_q)
{
    //initialize Task1,3 (direct teaching)
    /* Implement here.*/

    direct_teaching_.initialization(current_q);

    task1_waypoints_.row(0) = current_q;
    task2_waypoints_.row(0) = current_q;

    prev_targetQ = current_q;

    Eigen::MatrixXd stack_waypoints = direct_teaching_.get_stack_waypoints();

    task1_waypoints_.row(1) = stack_waypoints.row(1);
    task2_waypoints_.row(1) = stack_waypoints.row(2);
}

void FSM::print_state()
{
    std::cout<<"Current state is ";
    switch(state)
    {
        case FSM_IDLE :
            std::cout<<"FSM_IDLE"<<std::endl; 
            break;
        case FSM_TASK1 :
            std::cout<<"FSM_TASK1"<<std::endl; 
            break;
        case FSM_TASK2 :
            std::cout<<"FSM_TASK2"<<std::endl; 
            break;
        case FSM_TASK3 :
            std::cout<<"FSM_TASK3"<<std::endl; 
            break;
    }
}

Eigen::VectorXd FSM::Idle(const char &key_input)
{   
    //define Idle
    Eigen::VectorXd targetQ(dof_); targetQ.setZero();
    /* Implement here.*/

    //define transition event
    /* Implement here.*/

    targetQ = prev_targetQ;

    if (key_input == 't') {
        state = FSM_TASK1;
    }

    prev_targetQ=targetQ;

    return targetQ;
}

// tracking task1_waypoints
Eigen::VectorXd FSM::Task1(const Eigen::VectorXd &current_q)
{
    //define Task1
    Eigen::VectorXd targetQ(dof_); targetQ.setZero();
    /* Implement here.*/

    targetQ = direct_teaching_.TrajectoryGeneration(task1_waypoints_);

    Eigen::VectorXd error = current_q-task1_waypoints_.row(1).transpose();

    if(error.norm() < 0.01){
        direct_teaching_.clear();
        task2_waypoints_.row(0) = current_q;

        state = FSM_TASK2;
    }

    prev_targetQ=targetQ;

    //define transition event
    /* Implement here.*/

    return targetQ;
}

// task space control mode
Eigen::VectorXd FSM::Task2(const char &key_input, const bool& is_key_updated, const Eigen::VectorXd &current_q, const int &fd_gpio_30, const int &fd_gpio_31)
{   
    //define Task2
    Eigen::VectorXd targetQ(dof_); targetQ.setZero();
    /* Implement here.*/

    Eigen::Vector3d task_command;

    ///////////////////////////////////// Teleoperation mode Start /////////////////////////////////////
    if(is_key_updated){ // Perform teleoperation only when a new keyboard input is received (is_key_upated=true)                
        if(key_input=='i') // Move to the initial position for teleoperation
        {
            targetQ=current_q; 
            prev_targetQ=targetQ; // Update prev_targetQ
        }
        else if(key_input=='r' || key_input=='f' || key_input=='w' || key_input=='s' || key_input=='d' || key_input=='a') // Move the end-effector based on the task command
        {   
            double position_resolution=0.01; // The position resolution for teleoperation is 1cm
            if(key_input == 'r') { 
                task_command<< 0, 0, position_resolution; /*Implement here*/ 
            } 
            else if(key_input == 'f') { 
                task_command<< 0, 0, -position_resolution; /*Implement here*/  
            } 
            else if(key_input == 'w') { 
                task_command<< 0, position_resolution, 0; /*Implement here*/ 
            } 
            else if(key_input == 's') { 
                task_command<< 0, -position_resolution, 0; /*Implement here*/ 
            } 
            else if(key_input == 'd') { 
                task_command<< position_resolution, 0, 0; /*Implement here*/ 
            } 
            else if(key_input == 'a') { 
                task_command<< -position_resolution, 0, 0; /*Implement here*/ 
            }
            Eigen::MatrixXd J=Kinematics::GetJacobianMatrix(current_q); // J is the Jacobian matrix for linear velocity represented in the world frame
            targetQ = current_q + (J.transpose()*(J*J.transpose()).inverse()) * task_command; /*Implement here*/ 
            prev_targetQ=targetQ; // Update prev_targetQ
        }
        else if(key_input=='q')
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
        else if(key_input=='l')
        {  
            //When the key input is 'l', if the light is currently off, turn it on. If it is currently on, turn it off
            is_light_on_ = !is_light_on_; /*Implement here. */ 

            if(is_light_on_ == true)
            {
                std::cout<<"light ON"<<std::endl;
            }
            else
            {
                std::cout<<"light OFF"<<std::endl;
            }

            gpio_fd_set_value(fd_gpio_31, is_light_on_);

            targetQ=prev_targetQ;
        }
        else if(key_input=='e'){ // End teleoperation
            task2_waypoints_.row(0) = current_q;
            state = FSM_TASK3;
        }
        else if(key_input=='x'){ // End teleoperation
            task1_waypoints_.row(0) = current_q;
            state = FSM_IDLE;
        }
        else // When the wrong keyboard input is received, the targetQ is the privious targetQ value
        {
            targetQ=prev_targetQ;
        }
    }            
    else{ // Before receiving the new keyboard input, the targetQ is the privious targetQ value
        targetQ=prev_targetQ;
    }        
    ///////////////////////////////////// Teleoperation mode END /////////////////////////////////////    

    //define transition event
    /* Implement here.*/

    return targetQ;
}

// tracking task2_waypoints
Eigen::VectorXd FSM::Task3(const Eigen::VectorXd &current_q)
{
    //define Task3
    Eigen::VectorXd targetQ(dof_); targetQ.setZero();
    /* Implement here.*/

    targetQ = direct_teaching_.TrajectoryGeneration(task2_waypoints_);

    Eigen::VectorXd error = current_q - task2_waypoints_.row(1).transpose();

    if(error.norm() < 0.01){
        direct_teaching_.clear();

        state = FSM_TASK2;
    }

    prev_targetQ = targetQ;

    //define transition event
    /* Implement here.*/

    return targetQ;
}