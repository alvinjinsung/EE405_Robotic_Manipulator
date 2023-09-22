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
#include "DirectTeaching.hpp"

DirectTeaching::DirectTeaching(int dof_args)
{
    NumWaypoints=3; // the number of waypoints
    dof=dof_args; // DOF of the manipulator
    stack_waypoints.resize(NumWaypoints, dof); // Each row in the stack_waypoints matrix represents a waypoint for direct teaching
    
    NumNodes=400; // the number of nodes 
    NodeIndex=0; // node index. The range of NodeIndex is 0 ~ (NumNodes-1)
    StartPointIndex=0; // Waypoint index of start point in trajectory generation. The range of StartPointIndex is 0 ~ (NumWaypoints-1) 
    EndPointIndex=1; // Waypoint index of end point in trajectory generation. The range of EndPointIndex is 0 ~ (Numwaypoints-1)

}

// Define stack_waypoints where each row represents a waypoint for direct teaching in joint space
// The first waypoint for direct teaching is the initial joint angle when the program runs
void DirectTeaching::initialization(const Eigen::VectorXd &current_q)
{
    // Save two user-defined waypoints to the stack_waypoints using the "waypoints.csv" file
    // Note that the first waypoint for direct teaching is the initial joint angle when the program runs
    stack_waypoints <<current_q(0), current_q(1), current_q(2), current_q(3), MatrixFileIO::openData("waypoints.csv");
    
    std::cout << "Each row represents a waypoint for direct teaching in joint space: \n" << stack_waypoints<< std::endl;
}

// Clear the variables for direct teaching
void DirectTeaching::clear()
{
    NodeIndex=0; 
    StartPointIndex=0; 
    EndPointIndex=1; 
}

// The input argument is the task_waypoints matrix where each row represents a waypoint for direct teaching in joint space
// The output argument is the targetQ vector which is the desired joint angle at the current time step
// The targetQ vector is calculated by using the 1st order polynomial interpolation between the start point and the end point
// The difference with lab5 is that if the end point is the last waypoint, the targetQ vector is set to the desired q.
Eigen::VectorXd DirectTeaching::TrajectoryGeneration(const Eigen::MatrixXd &task_waypoints)
{
    Eigen::VectorXd targetQ(dof); targetQ.setZero();
    int num_task_waypoints = task_waypoints.rows();

    for(int j=0; j<dof; j++){
        targetQ(j)= (task_waypoints(EndPointIndex, j) - task_waypoints(StartPointIndex, j))/NumNodes*NodeIndex + task_waypoints(StartPointIndex, j); /* Implement here. targetQ(j) is the result of the first order polynomial interpolation 
        between task_waypoints(StartPointIndex, j) and task_waypoints(EndPointIndex, j) */
    }

    // If the node index is greater than or equal to the number of nodes,
    if(NodeIndex>=NumNodes)
    { 
        if(EndPointIndex>=num_task_waypoints-1) // If the end point is the last waypoint
        {   
            targetQ = task_waypoints.row(EndPointIndex); /* Implement here. Set the targetQ as the final waypoint */
        }
        else // If the end point is not the last waypoint
        {   
            //update the 'StartPointIndex' and the 'EndPointIndex' and reset the 'NodeIndex'
            StartPointIndex++; /* Implement here*/
            EndPointIndex++; /* Implement here*/
            NodeIndex = 0; /* Implement here*/
        }
    }
    else
    {   
        NodeIndex++; 
    }

    return targetQ;
}

// Return stack_waypoints
Eigen::MatrixXd DirectTeaching::get_stack_waypoints()
{
    return stack_waypoints;
}