/**
*\file aclient.cpp
*\brief implementation of action client, subscriber and publisher to deal with goal, velocity and position
*\author Alberto Di Donna
*\version 0.1
*\date 27/02/2024
*
*\details
*
*Subscribes to:<BR>
*	/odom
*
*Publishes to:<BR>
*	/Pos_vel
*
*Clients:<BR>
*	/reaching_goal
*
*Description:<BR>
This node implements an action client allowing the user to set a goal (x,y) or to cancel it. The node also publishes the robot position and velocity as a custom message (x,y,vel_x,vel_z), by relying on the values published on the topic /odom
**/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <iostream>
#include <thread>
#include <assignment_2_2023/Pos_vel.h>
#include <nav_msgs/Odometry.h>
#include<string>


ros::Publisher pub; ///< declaration of the publisher


/**
*\brief This function allows the user to cancel the target
*\param actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>& ac
**/
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


/**
*\brief This function allows the user to set the goal
*\param actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>& ac
**/
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


/**
*\brief This function publishes the informations related to velocity and position
*\param const nav_msgs::Odometry::ConstPtr& msg
**/
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


/**
*\brief The main function initialize the node, the subscriber, the publisher and the action client
*\param int argc
+\param char **argv
**/
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

