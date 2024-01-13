#include <ros/ros.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Last_target.h>


float last_goal_x;
float last_goal_y;


void last_target_callbck(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
	last_goal_x = goal->goal.target_pose.pose.position.x;
	last_goal_y = goal->goal.target_pose.pose.position.y;

}


bool last_target(assignment_2_2023::Last_target::Request& req, assignment_2_2023::Last_target::Response& res)
{ 

	res.x = last_goal_x; 
	res.y = last_goal_y;
	
	return true;
	
}



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
