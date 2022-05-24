#! /usr/bin/env python
# coding=UTF-8
"""
    Python 版 HelloWorld
"""

from time import sleep
import rospy
import math
import numpy
import nav_msgs.msg
import geometry_msgs.msg
import tf
from tf.transformations import euler_from_quaternion



# 定义全局变量
x = 0.0
y = 0.0
w_o = 0.0
x_o = 0
y_o = 0
z_o = 0
i = 0
X_t = 0.0
Y_t = 0.0
X_t_Pre = 0.0
Y_t_Pre = 0.0
r = 0
yaw_t = 0
liner_speed = 0
angular_speed = 0
liner_speed_old = 0
angular_speed_old = 0

def openreadtxt(file_name):
    data = []
    file = open(file_name, "r")
    file_data = file.readlines()
    for row in file_data:
        tmp_list = row.split(' ') #按‘，’切分每行的数据
        tmp_list[-1] = tmp_list[-1].replace('\n','') #去掉换行符
        data.append(tmp_list) #将每行数据插入data中
    file.close()
    data=numpy.array(data,dtype=float)   #将其转换成numpy的数组，并定义数据类型为int
    return data
def robot_rotation(desire_x,desire_y):
    while True:
        try:
            (trans1_map,rot1_map) = listener.lookupTransform('/map', '/robot3/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            (trans1_odom,rot1_odom) = listener.lookupTransform('/robot3/odom', '/robot3/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        x = trans1_map[0]
        y = trans1_map[1]
        (roll, pitch, yaw) = euler_from_quaternion([rot1_odom[0], rot1_odom[1], rot1_odom[2], rot1_odom[3]])
        if yaw < 0:
            yaw = yaw + 2 * math.pi
        print(x,y)

        X_t = desire_x
        Y_t = desire_y
        print(X_t,Y_t)
        
        # 判断坐标象限
        if (Y_t - y) == 0 and (X_t - x) > 0:
            yaw_t = 1.5 * math.pi
        if (Y_t - y) > 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 1.5 * math.pi
        if (Y_t - y) > 0 and (X_t - x) == 0:
            yaw_t = 0.0
        if (Y_t - y) > 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 0.5 * math.pi
        if (Y_t - y) == 0 and (X_t - x) < 0:
            yaw_t = 0.5 * math.pi
        if (Y_t - y) < 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 0.5 * math.pi
        if (Y_t - y) < 0 and (X_t - x) == 0:
            yaw_t = math.pi
        if (Y_t - y) < 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 1.5 * math.pi

        Theta_err = yaw_t - yaw

        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi

        abs_Theta_err = math.sqrt(pow(Theta_err,2))
        if abs_Theta_err < 0.01:
            break

        print(yaw_t,yaw)
        print("abs_Theta_err :", abs_Theta_err)

        liner_speed = 0.0
        angular_speed = 2.0 * Theta_err
        if angular_speed > 2.0:
            angular_speed = 2.0
        
        msg.linear.x = liner_speed
        msg.angular.z = angular_speed

        turtle_vel.publish(msg)                 # 向/cmd_vel话题发布数据


if __name__ == '__main__':

    rospy.init_node('item3')

    turtle_vel = rospy.Publisher('/robot3/cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size=1)
    # 3.创建 TF 订阅对象

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    data = openreadtxt('/home/rainbow/agv_path.txt')


    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Twist()

        while True:     # 根据自己mat数据的大小决定

            try:
                (trans1_map,rot1_map) = listener.lookupTransform('/map', '/robot3/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (trans1_odom,rot1_odom) = listener.lookupTransform('/robot3/odom', '/robot3/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            x = trans1_map[0]
            y = trans1_map[1]
            (roll, pitch, yaw) = euler_from_quaternion([rot1_odom[0], rot1_odom[1], rot1_odom[2], rot1_odom[3]])
            if yaw < 0:
                yaw = yaw + 2 * math.pi
            print(x,y)

            X_t = data[i][4]
            Y_t = data[i][5]
            print(X_t,Y_t)
            while X_t_Pre == X_t and Y_t_Pre == Y_t:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                turtle_vel.publish(msg)
                print("坐标未发生改变")
                sleep(2)
                i = i + 1
                if i >= 110:
                    print("小车已到达终点")
                    break                
                print(i)
                X_t = data[i][0]
                Y_t = data[i][1]

            if i == 21:
                print("原地掉头")
                robot_rotation(42,24)
            if i == 60:
                print("原地掉头")
                sleep(7)
                robot_rotation(24,38)
                print("掉头完成")
                i = i + 1
                print(i)            

            D_err = math.sqrt(math.pow((X_t - x), 2) + math.pow((Y_t - y), 2))
            # print(D_err)
            
            # 判断坐标象限
            if (Y_t - y) == 0 and (X_t - x) > 0:
                yaw_t = 1.5 * math.pi
            if (Y_t - y) > 0 and (X_t - x) > 0:
                yaw_t = math.atan((Y_t - y) / (X_t - x)) + 1.5 * math.pi
            if (Y_t - y) > 0 and (X_t - x) == 0:
                yaw_t = 0.0
            if (Y_t - y) > 0 and (X_t - x) < 0:
                yaw_t = math.atan((Y_t - y) / (X_t - x)) + 0.5 * math.pi
            if (Y_t - y) == 0 and (X_t - x) < 0:
                yaw_t = 0.5 * math.pi
            if (Y_t - y) < 0 and (X_t - x) < 0:
                yaw_t = math.atan((Y_t - y) / (X_t - x)) + 0.5 * math.pi
            if (Y_t - y) < 0 and (X_t - x) == 0:
                yaw_t = math.pi
            if (Y_t - y) < 0 and (X_t - x) > 0:
                yaw_t = math.atan((Y_t - y) / (X_t - x)) + 1.5 * math.pi

            Theta_err = yaw_t - yaw
            print(yaw_t,yaw)
            print("Theta_err :", Theta_err)

            if Theta_err < -math.pi:
                Theta_err = Theta_err + 2 * math.pi
            if Theta_err > math.pi:
                Theta_err = Theta_err - 2 * math.pi

            # print("Theta_err :", Theta_err)

            liner_speed = 1.0
            angular_speed = 3.5 * Theta_err
            if angular_speed > 3.5:
                angular_speed = 3.5
            
            msg.linear.x = liner_speed
            msg.angular.z = angular_speed

            liner_speed_old = liner_speed
            angular_speed_old = angular_speed

            turtle_vel.publish(msg)                 # 向/cmd_vel话题发布数据

            if D_err < 0.6:
                X_t_Pre = X_t
                Y_t_Pre = Y_t
                i = i + 1
                print(i)

            if i >= 110:
                print("小车已到达终点")
                break                         

            rate.sleep()  # 以固定频率执行

        # 如果到达最后一个点，让小车停下来
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print(msg.linear.x)

        turtle_vel.publish(msg)                 # 向/cmd_vel话题发布数据

        rate.sleep()  # 以固定频率执行
    rospy.spin()  # 保持节点运行，直到节点关闭