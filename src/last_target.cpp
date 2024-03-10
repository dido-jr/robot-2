/**
*\file last_target.cpp
*\brief service node to retrieve informations of the target
*\author Alberto Di Donna
*\version 0.1
*\date 27/02/2024
*
*\details
*
*Subscribes to:<BR>
*	/reaching_goal/goal
*
*Services:<BR>
*	/last_target
*
*Description:<BR>
*
*Service node that, when called, returns the coordinates of the last target sent by the user
**/



#include <ros/ros.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Last_target.h>


float last_goal_x; ///<x coordinate of the last goal
float last_goal_y; ///<y coordinate of the last goal

/**
*\brief this is the callback function of the subscriber that acces to the coordinates of the last goal and save them in the global variables
*\param const assignment_2_2023::PlanningActionGoal::ConstPtr& goal 
**/
void last_target_callbck(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
	last_goal_x = goal->goal.target_pose.pose.position.x;
	last_goal_y = goal->goal.target_pose.pose.position.y;

}

/**
*\brief this function write in the response part of the service the coordinates of the last goal
*\param assignment_2_2023::Last_target::Request& req is the request part of the service
*\param assignment_2_2023::Last_target::Response& res is the response part of the service
**/
bool last_target(assignment_2_2023::Last_target::Request& req, assignment_2_2023::Last_target::Response& res)
{ 

	res.x = last_goal_x; 
	res.y = last_goal_y;
	
	return true;
	
}


/**
*\brief the main function initialize the node, the subscriber, the service and retrieve the parameters
*\param int argc
*\param char **argv
**/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "last_target");
	ros::NodeHandle nh;
	
	ros::param::get("/des_pos_x", last_goal_x);
	ros::param::get("/des_pos_y", last_goal_y);
	
	ros::Subscriber sub = nh.subscribe<assignment_2_2023::PlanningActionGoal>("/reaching_goal/goal",1, last_target_callbck);
	
	ros::ServiceServer service = nh.advertiseService("/last_target", last_target);
	
	ros::spin();
	return 0;
}
