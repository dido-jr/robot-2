#include <ros/ros.h>
#include <assignment_2_2023/Pos_vel.h>
#include <assignment_2_2023/Dist_speed.h>
#include <assignment_2_2023/PlanningAction.h>
#include <cmath>


float goal_x =0.0, goal_y=0.0;
float x, y;
float vel_x_m = 0.0, vel_z_m;
int count = 0;
int N = 0;
float sum_x = 0.0, sum_z = 0.0;


void pos_vel_cllbck(const assignment_2_2023::Pos_vel::ConstPtr& msg)
{
	//actual pose and vel from custom msg
	x = msg->x;
	y = msg->y;
	float vel_x = msg->vel_x;
	float vel_z = msg->vel_z;
	
	if (count < N)
	{
		sum_x += vel_x;
		sum_z += vel_z;
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


void goal_callbck(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
	//taking goal coordinate
	goal_x = goal->goal.target_pose.pose.position.x;
	goal_y = goal->goal.target_pose.pose.position.y;
}


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
