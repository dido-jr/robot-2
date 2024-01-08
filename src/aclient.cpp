#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <iostream>
#include <thread>
#include <assignment_2_2023/Pos_vel.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub;

void cancelGoal(actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>& ac)
{
	char userInput = ' ';
	std::cout<< "Do you want to cancel the target? [y/n] ";
	std::cin>>userInput;
	std::cout<<userInput;
	//while (userInput != 'n' || userInput != 'y' )
	//{
	//	std::cout<<"Insert a valid input: 'y' if you want to cancel the target, 'n' otherwise";
	//}
	if (userInput == 'y')
	{
		ac.cancelGoal();
		ROS_INFO("Goal has been cancelled");
		std::terminate();
	}

	
}


void Pos_vel_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	assignment_2_2023::Pos_vel pos_vel;
	pos_vel.x = msg->pose.pose.position.x;
	pos_vel.y = msg->pose.pose.position.y;
	pos_vel.vel_x = msg->twist.twist.linear.x;
	pos_vel.vel_z = msg->twist.twist.angular.z;
	pub.publish(pos_vel);
}


int main(int argc, char **argv)
{
	//initialization
	ros::init(argc, argv, "aclient");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/odom",1,Pos_vel_callback);
	pub = nh.advertise<assignment_2_2023::Pos_vel>("/Pos_vel",1);
	
	actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("/reaching_goal",true);
	ac.waitForServer();
	
	//goal
	assignment_2_2023::PlanningGoal goal;
	
	std::cout<<"Enter x coordinate: ";
	std::cin>>goal.target_pose.pose.position.x;
	
	std::cout<<"Enter y coordinate: ";
	std::cin>>goal.target_pose.pose.position.y;
	
	ac.sendGoal(goal);

	//cancel goal
	std::thread cancelGoalThread(cancelGoal, std::ref(ac));
	
	//status
	ac.waitForResult();
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Goal reached: %s", state.toString().c_str());
	
	
	ros::spin();
	return 0;
	
}
