#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion
import math
 
class Catman:
 
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('Catman_controller', anonymous=True)
 
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher("/Maran/thruster_manager/input", geometry_msgs.msg.Wrench, queue_size=1)
        self.flag=0
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber("/Maran/gps",sensor_msgs.msg.NavSatFix,self.update_pose)
        self.angle_sub=rospy.Subscriber("/Maran/pose_gt",nav_msgs.msg.Odometry,self.update_angle)
        self.angle=nav_msgs.msg.Odometry()
        self.pose = sensor_msgs.msg.NavSatFix()
        self.rate = rospy.Rate(10)
        self.angle
        self.sum_error=0
        self.Cu=120
        self.Cp=0.6*self.Cu
        self.Ci=2.2*self.Cp/self.Cu
        self.Cd=self.Cp*self.Cu/8
        self.Flag1=0
        self.prev_distance_er=0
        self.distance_er_summ=0
        self.Cu_speed=100;
        self.Cp_speed=0.6*self.Cu_speed
        self.Ci_speed=2*self.Cp_speed/self.Cu_speed
        self.Cd_speed=self.Cp_speed*self.Cu_speed/8
    def update_angle(self,data):
        rot_q=data.pose.pose.orientation
        (roll,pitch,theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.angle=theta
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.longitude = float("{0:.6f}".format(data.longitude))
        self.pose.latitude = float("{0:.6f}".format(data.latitude))
 
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        b=sqrt(pow(goal_pose.longitude - abs(self.pose.longitude), 2) +
                    pow(goal_pose.latitude - abs(self.pose.latitude), 2))
        if self.Flag1 == 0:
            self.prev_distance_er=float("{0:.6f}".format(b))
            self.Flag1 = 1
        print(goal_pose.longitude, self.pose.longitude, goal_pose.latitude, self.pose.latitude)
        return b
 
    def linear_vel(self, goal_pose, constant=100000):
        print(float("{0:.6f}".format(self.euclidean_distance(goal_pose))))
        self.Et=float("{0:.6f}".format(self.euclidean_distance(goal_pose)))
        self.Pt_speed=self.Cp_speed*self.Et
        self.It_speed=self.distance_er_summ+self.Ci_speed*self.Et
        self.Dt_speed=self.Cd_speed+self.prev_distance_er
        self.prev_distance_er=self.Et
        self.distance_er_summ=self.It_speed
        return int(self.Pt_speed+self.It_speed+self.Dt_speed)
 
    def steering_angle(self, goal_pose):
        
        p=atan2(goal_pose.latitude - abs(self.pose.latitude), goal_pose.longitude - abs(self.pose.longitude))
       
        if self.flag==0:
            self.prev_angle_er=abs(self.angle)-abs(p)
            self.flag=1
        return p
 
    def angular_vel(self, goal_pose, constant=10):
        self.et=self.steering_angle(goal_pose) - self.angle
        self.Pt=self.Cp*self.et
        self.It=self.sum_error+self.Ci*self.et
        self.Dt=self.Cd*self.prev_angle_er
        self.sum_error=self.It
        self.prev_angle_er=self.et
        return int(self.Pt+self.It+self.Dt)
        
 
    def move2goal(self):
        goal_pose = sensor_msgs.msg.NavSatFix()
        # Get the input from the user.
        goal_pose.latitude = input("Set your x goal: ")
        goal_pose.longitude = input("Set your y goal: ")
 
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")
 
        vel_msg = geometry_msgs.msg.Wrench()
 
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
 
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
 
            # Linear velocity in the x-axis.
            vel_msg.force.x = self.linear_vel(goal_pose)
            vel_msg.force.y = 0
            vel_msg.force.z = 0
 
            # Angular velocity in the z-axis.
            vel_msg.torque.x = 0
            vel_msg.torque.y = 0
            vel_msg.torque.z = self.angular_vel(goal_pose)
            print(self.linear_vel(goal_pose), self.angular_vel(goal_pose))
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
 
            # Publish at the desired rate.
            self.rate.sleep()
 
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
 
        # If we press control + C, the node will stop.
        rospy.spin()
 
if __name__ == '__main__':
    try:
        x = Catman()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
