#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int8.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#define PI 3.1415926
#define Desire_Delta_X 0.5
#define Desire_Delta_Y 0.5
vector<geometry_msgs::Point>path2;

ros::Publisher robot2_vel;
geometry_msgs::Twist vel_msg;
geometry_msgs::Twist robot1_Vel_msg;

tf::StampedTransform transform2_map;
tf::StampedTransform transform2_odom;
tf::StampedTransform transform1_map;

void robot1_VelCallBack(const geometry_msgs::Twist & Vel_msg); 

int main(int argc, char** argv){

    ros::init(argc, argv, "formation_robot2");
    ros::NodeHandle node;

    robot2_vel =
        node.advertise<geometry_msgs::Twist>("/robot2/cmd_vel_mux/input/teleop", 10);

    ros::Subscriber robot1Vel = node.subscribe("/robot1/cmd_vel_mux/input/teleop", 1, robot1_VelCallBack);

    tf::TransformListener listener;

    try{
      listener.waitForTransform("/map","/robot2/base_link",
                                  ros::Time(0), ros::Duration(0.01));
      listener.lookupTransform("/map", "/robot2/base_link",
                                  ros::Time(0), transform2_map);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    try{
      listener.waitForTransform("/robot2/odom","/robot2/base_link",
                              ros::Time(0), ros::Duration(0.01));
      listener.lookupTransform("/robot2/odom", "/robot2/base_link",
                              ros::Time(0), transform2_odom);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    try{
        listener.waitForTransform("/map","/robot1/base_link",
                                    ros::Time(0), ros::Duration(0.01));
        listener.lookupTransform("/map", "/robot1/base_link",
                                    ros::Time(0), transform1_map);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    double x_desire,y_desire,robot_x,robot_y;
    double delta_x,delta_y;
    double yaw_desire,robot_yaw;
    double liner_speed,angular_speed,liner_speed_old,angular_speed_old;
    double D_err = 1000.0,Theta_err,real_D_err;
    double current_x;
    int i =0;

    ros::Rate rate(1000.0);
    while (node.ok()){
        try{
          listener.waitForTransform("/map","/robot2/base_link",
                                      ros::Time(0), ros::Duration(0.01));
          listener.lookupTransform("/map", "/robot2/base_link",
                                      ros::Time(0), transform2_map);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        //From Slave to master transformation
        try{
          listener.waitForTransform("/robot2/odom","/robot2/base_link",
                                  ros::Time(0), ros::Duration(0.01));
          listener.lookupTransform("/robot2/odom", "/robot2/base_link",
                                  ros::Time(0), transform2_odom);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        try{
            listener.waitForTransform("/map","/robot1/base_link",
                                        ros::Time(0), ros::Duration(0.01));
            listener.lookupTransform("/map", "/robot1/base_link",
                                        ros::Time(0), transform1_map);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        current_x = transform1_map.getOrigin().x();

        if(current_x < 4.0){
            x_desire = transform1_map.getOrigin().x() - Desire_Delta_X;
            y_desire = transform1_map.getOrigin().y() + Desire_Delta_Y;          
        }
        if(current_x > 4.0 & current_x < 9.0){
            x_desire = transform1_map.getOrigin().x() - 0.6;
            y_desire = transform1_map.getOrigin().y();
        }

        
        robot_x = transform2_map.getOrigin().x();
        robot_y = transform2_map.getOrigin().y();
        delta_x = x_desire - robot_x;
        delta_y = y_desire - robot_y;
            
        robot_yaw = tf::getYaw(transform2_odom.getRotation());
        if(robot_yaw < 0)
            robot_yaw = robot_yaw + 2 * PI;

        if(delta_y == 0 & delta_x > 0)
            yaw_desire = 0.0;
        else if(delta_y > 0 & delta_x > 0)
            yaw_desire = atan(delta_y/delta_x);
        else if(delta_y > 0 & delta_x == 0)
            yaw_desire = 0.5 * PI;
        else if(delta_y > 0 & delta_x < 0)
            yaw_desire = atan(delta_y/delta_x) + 1.0 *PI;
        else if(delta_y == 0 & delta_x < 0)
            yaw_desire = 1.0 * PI;
        else if(delta_y < 0 & delta_x < 0)
            yaw_desire = atan(delta_y/delta_x) + 1.0 * PI;
        else if(delta_y < 0 & delta_x == 0)
            yaw_desire = 1.5 * PI;
        else if(delta_y < 0 & delta_x > 0)
            yaw_desire = atan(delta_y/delta_x) + 2.0 * PI;

        Theta_err = yaw_desire - robot_yaw;
        if(Theta_err < -PI)
            Theta_err = Theta_err + 2 * PI;
        if(Theta_err > PI)
            Theta_err = Theta_err - 2 * PI;
        // std::cout << "yaw_desire,robot_yaw:" << yaw_desire << "," << robot_yaw << std::endl;
        // std::cout << "Theta_err :" << Theta_err << std::endl;
        if(current_x > 2.0 & robot1_Vel_msg.linear.x == 0){
            std::cout << "robot2到达终点" << std::endl;
            break;
        }

        liner_speed = robot1_Vel_msg.linear.x + 0.5 * delta_x;

        angular_speed = 2.5 * Theta_err;
        if(angular_speed > 2.0)
            angular_speed = 2.0;   

        vel_msg.linear.x = liner_speed;
        vel_msg.angular.z = angular_speed;
        robot2_vel.publish(vel_msg);    //发布运动指令

        ros::spinOnce();
        rate.sleep();
    }
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    robot2_vel.publish(vel_msg);    //发布运动指令
    return 0;
};

/*
 * Call back implementation to read and process robot1_Vel data  
 */
void robot1_VelCallBack(const geometry_msgs::Twist & Vel_msg)
{
  robot1_Vel_msg.linear.x = Vel_msg.linear.x;
}
