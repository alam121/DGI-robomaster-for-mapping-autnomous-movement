#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import LaserScan



from pynput import keyboard

import robomaster
from robomaster import robot
import time
import pandas as pd
from pandas import *
from matplotlib import pyplot as plt
import time
import math
from math import *

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg



ir1 = 0.000
ir2 = 0.000
ir3 = 0.000


def sub_data_handler(sub_info):
    global ir1,ir2,ir3
    
    distance = sub_info
    ir1=distance [0]
    ir2=distance [1]
    ir3=distance [2]
    

def sub_attitude_info_handler(attitude_info):
    global yaw,pitch,roll 
    yaw, pitch, roll = attitude_info
    
    yaw = yaw*(math.pi/180)
    #yaw=atan2(sin(yaw),cos(yaw))  
   
    
 #   print("chassis attitude: yaw:{0}".format(yaw))


def sub_position_handler(position_info):
    global x,y,z
   
   
    
    x, y, z = position_info
    
    
    
#    print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))


    



if __name__ == '__main__':

    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    ep_chassis.sub_position(freq=50, callback=sub_position_handler)
    
    ep_sensor.sub_distance(freq=50, callback=sub_data_handler)

    ep_chassis.sub_attitude(freq=50, callback=sub_attitude_info_handler)
    
    
    
    rospy.init_node('scan_publisher')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size =50)
    
    #r = rospy.Rate(50)
      
    num_readings = 3
    laser_frequency = 10
    
    
    scan = LaserScan()
 
    scan.intensities = []
    intensity = 100 	
    count = 0
    
    initial = tf.transformations.quaternion_from_euler(0,0,yaw)
    
    while(1):
      

        now = rospy.get_rostime()      
        current_time = now
        scan.header.stamp = current_time
        scan.header.frame_id = 'base_link'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = math.pi / 2
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 5.0
        scan.scan_time  = (1 / laser_frequency)
        scan.ranges = []
        
        #scan.ranges.append(tuple([ir1,ir2,ir3]))
        
        scan.intensities.append(intensity+count)
        count = count + 1
        
        scan.ranges.append(ir1/1000)  
        scan.ranges.append(ir2/1000) 
        scan.ranges.append(ir3/1000)         
        
        scan_pub.publish(scan)
        
        with keyboard.Events() as events:
            
            event = events.get(1e6)
            
            if event.key == keyboard.KeyCode.from_char('w'):
                #scan_pub.publish(scan)
                ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.5).wait_for_completed()
                #scan_pub.publish(scan)
    	    #continue
    	    
            elif event.key == keyboard.KeyCode.from_char('s'):
    	            #scan_pub.publish(scan)
    	        ep_chassis.move(x=-0.05, y=0, z=0, xy_speed=0.5).wait_for_completed()
    	            #scan_pub.publish(scan)
    	    #continue
            elif event.key == keyboard.KeyCode.from_char('d'):
                    #scan_pub.publish(scan)
                ep_chassis.move(x=0, y=0, z=-5, xy_speed=0.5).wait_for_completed()
                    #scan_pub.publish(scan)
    	    #continue

            elif event.key == keyboard.KeyCode.from_char('a'):
                    #scan_pub.publish(scan)
                ep_chassis.move(x=0, y=0, z=5, xy_speed=0.5).wait_for_completed()
                    #scan_pub.publish(scan)
    	    #continue

            elif event.key == keyboard.KeyCode.from_char('q'):
                    #scan_pub.publish(scan)
                ep_chassis.move(x=0.4, y=0, z=5, xy_speed=0.5).wait_for_completed()
                    #scan_pub.publish(scan)
    	    #continue

            elif event.key == keyboard.KeyCode.from_char('z'):
                    #scan_pub.publish(scan)
                ep_chassis.move(x=-0.4, y=0, z=5, xy_speed=0.5).wait_for_completed()
                    #scan_pub.publish(scan)
    	    #continue

            elif event.key == keyboard.KeyCode.from_char('l'):
    	        break
    	    
      
        #tf_func()        
        broadcaster = tf2_ros.TransformBroadcaster()
        broadcaster_static = tf2_ros.StaticTransformBroadcaster()
        
        
        #.......................odom--->map................
        
        
                
        #static_transformStamped_map = geometry_msgs.msg.TransformStamped()
        #static_transformStamped_map.header.stamp = rospy.Time.now()
        #static_transformStamped_map.header.frame_id = "map"
        #static_transformStamped_map.child_frame_id = 'odom'

        #static_transformStamped_map.transform.translation.x = 0
        #static_transformStamped_map.transform.translation.y = 0
        #static_transformStamped_map.transform.translation.z = 0

        #quat = tf.transformations.quaternion_from_euler(0.0,0.0,-yaw)
    
        #static_transformStamped_map.transform.rotation.x = 0
        #static_transformStamped_map.transform.rotation.y = 0
        #static_transformStamped_map.transform.rotation.z = 1
        #static_transformStamped_map.transform.rotation.w = 0
     
        #-------------odom->baselink-----------------------------
        
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "odom"
        static_transformStamped.child_frame_id = 'base_link'

        static_transformStamped.transform.translation.x = float(x)
        static_transformStamped.transform.translation.y = float(y)
        static_transformStamped.transform.translation.z = float(z)

        quat = tf.transformations.quaternion_from_euler(0.0,0.0,-yaw)
    
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        
                
                #----------------laser-> base ink----------------------
                
        static_transformStamped_lb = geometry_msgs.msg.TransformStamped()

        static_transformStamped_lb.header.stamp = rospy.Time.now()
        static_transformStamped_lb.header.frame_id = "base_link"
        static_transformStamped_lb.child_frame_id = 'base_laser'

        static_transformStamped_lb.transform.translation.x = 0.0
        static_transformStamped_lb.transform.translation.y = 0.0
        static_transformStamped_lb.transform.translation.z = 0.0

    
        static_transformStamped_lb.transform.rotation.x = 1.0
        static_transformStamped_lb.transform.rotation.y = 0.0
        static_transformStamped_lb.transform.rotation.z = 0.0
        static_transformStamped_lb.transform.rotation.w = 0.0
        
        
        #.......................Right front wheel 1................
        
        static_transformStamped_rf1 = geometry_msgs.msg.TransformStamped()

        static_transformStamped_rf1.header.stamp = rospy.Time.now()
        static_transformStamped_rf1.header.frame_id = "base_link"
        static_transformStamped_rf1.child_frame_id = 'front_right_wheel_link'

        static_transformStamped_rf1.transform.translation.x = -0.1
        static_transformStamped_rf1.transform.translation.y = 0.1
        static_transformStamped_rf1.transform.translation.z = 0.1

        
        static_transformStamped_rf1.transform.rotation.x = initial[0]
        static_transformStamped_rf1.transform.rotation.y = initial[1]
        static_transformStamped_rf1.transform.rotation.z = initial[2]
        static_transformStamped_rf1.transform.rotation.w = initial[3]
        
        
        
         #.......................left front wheel 2................
        
        static_transformStamped_lf2 = geometry_msgs.msg.TransformStamped()

        static_transformStamped_lf2.header.stamp = rospy.Time.now()
        static_transformStamped_lf2.header.frame_id = "base_link"
        static_transformStamped_lf2.child_frame_id = 'front_left_wheel_link'

        static_transformStamped_lf2.transform.translation.x = 0.1
        static_transformStamped_lf2.transform.translation.y = -0.1
        static_transformStamped_lf2.transform.translation.z = 0.1
  
        static_transformStamped_lf2.transform.rotation.x = initial[0]
        static_transformStamped_lf2.transform.rotation.y = initial[1]
        static_transformStamped_lf2.transform.rotation.z = initial[2]
        static_transformStamped_lf2.transform.rotation.w = initial[3]       
        


         #.......................left rear wheel 3................
        
        static_transformStamped_lr3 = geometry_msgs.msg.TransformStamped()

        static_transformStamped_lr3.header.stamp = rospy.Time.now()
        static_transformStamped_lr3.header.frame_id = "base_link"
        static_transformStamped_lr3.child_frame_id = 'rear_left_wheel_link'

        static_transformStamped_lr3.transform.translation.x = 0.1
        static_transformStamped_lr3.transform.translation.y = 0.1
        static_transformStamped_lr3.transform.translation.z = 0.1
  
        static_transformStamped_lr3.transform.rotation.x = initial[0]
        static_transformStamped_lr3.transform.rotation.y = initial[1]
        static_transformStamped_lr3.transform.rotation.z = initial[2]
        static_transformStamped_lr3.transform.rotation.w = initial[3]      
        


         #.......................right rear wheel 4................
        
        static_transformStamped_rr4 = geometry_msgs.msg.TransformStamped()

        static_transformStamped_rr4.header.stamp = rospy.Time.now()
        static_transformStamped_rr4.header.frame_id = "base_link"
        static_transformStamped_rr4.child_frame_id = 'rear_right_wheel_link'

        static_transformStamped_rr4.transform.translation.x = -0.1
        static_transformStamped_rr4.transform.translation.y = 0.1
        static_transformStamped_rr4.transform.translation.z = 0.1
  
        static_transformStamped_rr4.transform.rotation.x = initial[0]
        static_transformStamped_rr4.transform.rotation.y = initial[1]
        static_transformStamped_rr4.transform.rotation.z = initial[2]
        static_transformStamped_rr4.transform.rotation.w = initial[3]      
        
        
        


        
  
        #print('x is ',x)
        #print('y is ',y)
        
        broadcaster.sendTransform([static_transformStamped,static_transformStamped_rf1,static_transformStamped_lf2,static_transformStamped_lr3, static_transformStamped_rr4])
  


        broadcaster_static.sendTransform([static_transformStamped_lb])
        #rospy.spin()



    
    print('done')
    

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()
    
    #print(pd)
    #r.sleep()
    ep_robot.close()
    
    

