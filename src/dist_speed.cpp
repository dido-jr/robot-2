#include <ros/ros.h>
#include <assignment_2_2023/Pos_vel.h>
#include <assignment_2_2023/Dist_speed.h>
#include <assignment_2_2023/PlanningAction.h>
#include <cmath>


float goal_x =0.0, goal_y=0.0;
float x, y, vel_x, vel_z;


void pos_vel_cllbck(const assignment_2_2023::Pos_vel::ConstPtr& msg)
{
	x = msg->x;
	y = msg->y;
	vel_x = msg->vel_x;
	vel_z = msg->vel_z;
}


void goal_callbck(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
	goal_x = goal->goal.target_pose.pose.position.x;
	goal_y = goal->goal.target_pose.pose.position.y;
}


bool dist_speed_cllbck(assignment_2_2023::Dist_speed::Response& res, assignment_2_2023::Dist_speed::Request& req)
{
	res.goal_dist = sqrt(pow(goal_y - y, 2) +
                        pow(goal_x - x, 2));
        res.velm_x = 0;
        res.velm_z = 0;
                        
        return 1;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dist_speed");
	ros::NodeHandle nh;
	
	ros::Subscriber sub1 = nh.subscribe("/Pos_vel",1, pos_vel_cllbck);
	
	ros::Subscriber sub2 = nh.subscribe<assignment_2_2023::PlanningActionGoal>("/reaching_goal/goal",1, goal_callbck);
	
	ros::ServiceServer service = nh.advertiseService("/dist_speed", dist_speed_cllbck);
	
	ros::spin();
	
	return 0;

}
