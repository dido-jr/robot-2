/**
*\file dist_speed.cpp
*\brief Service node to retrieve informations about the position and the speed of the robot
*\author Alberto Di Donna
*\version 0.1
*\date 27/02/2024
*
*\details
*
*Subscribes to:<BR>
*	/Pos_vel <BR>
*	/reaching_goal/goal
*
*Service:<BR>
*	/dist_speed
*
*Description:<BR>
Service node that subscribes to the robot's position and velocity and implements a server to retrieve the distance of the robot from the target and the robt's average speed
**/

#include <ros/ros.h>
#include <assignment_2_2023/Pos_vel.h>
#include <assignment_2_2023/Dist_speed.h>
#include <assignment_2_2023/PlanningAction.h>
#include <cmath>


float goal_x =0.0; ///<x coordinate of the goal
float goal_y=0.0; ///<y coordinate of the goal
float x; ///<x coordinate of the robot
float y; ///<y coordinate of the robot
float vel_x_m = 0.0;///<average x speed of the robot
float vel_z_m;///<average z speed of the robot
int count = 0;///<counter
int N = 0;///<time interval on which compute the average speed
float sum_x = 0.0;///<sum of x speed at each instant
float sum_z = 0.0;///<sum of z speed at each instant


/**
*\brief This function compute the average speed of the robot
*\param float vel__x
*\param float vel__z 
**/
void average_speed(float vel__x, float vel__z)
{
	if (count < N)
	{
		sum_x += vel__x;
		sum_z += vel__z;
	}
	else if (count == N)
	{
		vel_x_m = sum_x / N;
		vel_z_m = sum_z / N;
	}
	else if (count > N)
	{
		count = 0;
		sum_x = 0.0;
		sum_z = 0.0;
		vel_x_m =0.0;
		vel_z_m =0.0;
	}
	count++;
}



/**
*\brief callback function that subscribes from the topic /Pos_vel to retrieve the actual position and velocity
*\param const assignment_2_2023::Pos_vel::ConstPtr& msg
**/
void pos_vel_cllbck(const assignment_2_2023::Pos_vel::ConstPtr& msg)
{
	//actual pose and vel from custom msg
	x = msg->x;
	y = msg->y;
	float vel_x = msg->vel_x;
	float vel_z = msg->vel_z;
	
	average_speed(vel_x, vel_z);
	
}


/**
*\brief callback function that subscribes from /reaching_goal/goal to retrieve the goal coordinates
*\param const assignment_2_2023::PlanningActionGoal::ConstPtr& goal
**/
void goal_callbck(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
	//taking goal coordinate
	goal_x = goal->goal.target_pose.pose.position.x;
	goal_y = goal->goal.target_pose.pose.position.y;
}


/**
*\brief callback function related to the service /dist_speed to publish the distance from goal and the average velocity
*\param assignment_2_2023::Dist_speed::Request& req
*\param assignment_2_2023::Dist_speed::Response& res
**/
bool dist_speed_cllbck(assignment_2_2023::Dist_speed::Request& req, assignment_2_2023::Dist_speed::Response& res)
{	
	//publishing distance from goal
	res.goal_dist = sqrt(pow(goal_y - y, 2) +
                        pow(goal_x - x, 2));
                        
        //and average velocity 
        std::cout<<vel_x_m;
        res.velm_x = vel_x_m;
        res.velm_z = vel_z_m;
 
                      
        return 1;
}


/**
*\brief the main function initialize the node, the subscribers, the service and retrieve the parameter
*\param int argc
*\param char **argv
**/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "dist_speed");
	ros::NodeHandle nh;
	
	
	ros::param::get("/time",N);
	
	ros::Subscriber sub1 = nh.subscribe("/Pos_vel",1, pos_vel_cllbck);
	
	ros::Subscriber sub2 = nh.subscribe<assignment_2_2023::PlanningActionGoal>("/reaching_goal/goal",1, goal_callbck);
	
	ros::ServiceServer service = nh.advertiseService("/dist_speed", dist_speed_cllbck);
	
	ros::spin();
    	
	return 0;

}
