#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <iostream>
#include <thread>
#include <assignment_2_2023/Pos_vel.h>
#include <nav_msgs/Odometry.h>
#include<string>

float pos_vel_x, pos_vel_y, pos_vel_vel_x, pos_vel_vel_z;

ros::Publisher pub;

void cancelGoal(actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>& ac)
{
    char userInput = ' ';
    std::cout<< "Do you want to cancel the target? [y/n] ";
    std::cin>>userInput;
    while (userInput != 'n' && userInput != 'y' )
    {
	std::cout<<"Insert a valid input: 'y' if you want to cancel the target, 'n' otherwise: ";
	std::cin>>userInput;
    }
    if (userInput == 'y' && ac.getState().toString()!="SUCCEEDED")
    {
    	
	ac.cancelGoal();
	ROS_INFO("Goal has been cancelled");

    }
    else if (userInput == 'y' && ac.getState().toString()=="SUCCEEDED")
    {
    	ROS_INFO("Goal already reached! Choose a new target ");
    }

	
}

void setcancelGoal(actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>& ac)
{
    while(1){
	//set goal
	assignment_2_2023::PlanningGoal goal;
	
	std::cout<<"Enter x coordinate: ";
	std::cin>>goal.target_pose.pose.position.x;
	
	std::cout<<"Enter y coordinate: ";
	std::cin>>goal.target_pose.pose.position.y;
	
	ac.sendGoal(goal);
	ROS_INFO("Reaching the goal...");

	//cancel goal
	std::thread cancelGoalThread(cancelGoal, std::ref(ac));

	//status
	ac.waitForResult();
	actionlib::SimpleClientGoalState state = ac.getState();
	std::string stato = state.toString();
	if (stato == "SUCCEEDED"){
	ROS_INFO("Goal reached: %s", state.toString().c_str());}
	
	cancelGoalThread.join();
    }
}

void Pos_vel_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
     //publishing the information received from odom
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
	
	//subscriber to robot position
	ros::Subscriber sub = nh.subscribe("/odom",1,Pos_vel_callback);
	
	//publisher set up
	pub = nh.advertise<assignment_2_2023::Pos_vel>("/Pos_vel",1000);
	
	//action client
	actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("/reaching_goal",true);
	
	ac.waitForServer();
	
	std::thread setcancelGoalThread(setcancelGoal, std::ref(ac));
	
	ros::spin();


     return 0;
	
}

