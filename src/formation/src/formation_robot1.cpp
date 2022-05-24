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
vector<geometry_msgs::Point>path1;

ros::Publisher robot1_vel;
geometry_msgs::Twist robot1_vel_msg;

tf::StampedTransform transform1_map;
tf::StampedTransform transform1_odom;

void readpath(void);




int main(int argc, char** argv){

    ros::init(argc, argv, "formation_robot1");
    ros::NodeHandle node;

    robot1_vel =
        node.advertise<geometry_msgs::Twist>("/robot1/cmd_vel_mux/input/teleop", 10);

    tf::TransformListener listener;

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
    try{
        listener.waitForTransform("/robot1/odom","/robot1/base_link",
                                ros::Time(0), ros::Duration(0.01));
        listener.lookupTransform("/robot1/odom", "/robot1/base_link",
                                ros::Time(0), transform1_odom);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    double x_desire,y_desire,robot_x,robot_y,delta_x,delta_y;
    double yaw_desire,robot_yaw;
    double liner_speed,angular_speed,liner_speed_old,angular_speed_old;
    double D_err = 1000.0,Theta_err;
    double current_x;
    int i =0;
    readpath();
    for(int i = 0;i<path1.size();i++)
    {
        std::cout<< "x=:" << path1[i].x << ",y=:" << path1[i].y << std::endl;
    }

    ros::Rate rate(1000.0);
    while (node.ok()){
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

        //From Slave to master transformation
        try{
            listener.waitForTransform("/robot1/odom","/robot1/base_link",
                                    ros::Time(0), ros::Duration(0.01));
            listener.lookupTransform("/robot1/odom", "/robot1/base_link",
                                    ros::Time(0), transform1_odom);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        current_x = transform1_map.getOrigin().x();
        std::cout << "current_x:" << current_x << std::endl;

        //avoid();//function call to avoid obstacles while following the Master

        x_desire = path1[i].x;
        y_desire = path1[i].y;
        std::cout << "x_desire:" << x_desire << std::endl;
        std::cout << "y_desire:" << y_desire << std::endl;
        robot_x = transform1_map.getOrigin().x();
        robot_y = transform1_map.getOrigin().y();
        delta_x = x_desire - robot_x;
        delta_y = y_desire - robot_y;

        D_err = sqrt(pow((delta_x), 2) + pow((delta_y), 2));
        // std::cout << "robot1_D_err :" << D_err << std::endl;
        if(D_err < 0.1){
            i = i + 1;
            std::cout << "i = " << i << std::endl;
            if(i >= (path1.size()-1)){
              std::cout << "robot1到达终点" << std::endl;
              break;
            }
        }

            
        robot_yaw = tf::getYaw(transform1_odom.getRotation());
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
        
        //目前先只使用最简单的比例控制

        liner_speed = 0.8;
        angular_speed = 3.0 * Theta_err;

        robot1_vel_msg.linear.x = liner_speed;
        robot1_vel_msg.angular.z = angular_speed;
        liner_speed_old = liner_speed;
        angular_speed_old = angular_speed;
    //      std::cout << "liner_speed :" << liner_speed << std::endl;
        robot1_vel.publish(robot1_vel_msg);    //发布运动指令

        ros::spinOnce();
        rate.sleep();
    }
        robot1_vel_msg.linear.x = 0.0;
        robot1_vel_msg.angular.z = 0.0;
        robot1_vel.publish(robot1_vel_msg);    //发布运动指令

    return 0;
};

void readpath(void){
    ifstream infile("/home/rainbow/multirobot_formation/src/Path/agv1_path.txt");
    string line;
     while (!infile.eof())
     {
         getline(infile,line);
         float x1,y1;
         geometry_msgs::Point Point;
         stringstream data(line);
         data>>x1>>y1;
         Point.x = x1,Point.y = y1;
         path1.push_back(Point);
     }
}
