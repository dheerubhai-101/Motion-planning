import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import rospy
from nav_msgs.msg import Path
from math import pow,atan2,sqrt
from math import sqrt
from std_msgs.msg import Float64
import math


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def joint_name(number):
    name= '/joint'+ str(number)+'_vel_controller/command'
    return name


pos = rospy.Subscriber("/ground_truth/state",Odometry, callback)

joints=[]
pub=[]
total_joints=4
rate= rospy.Rate(1)

for i in range(total_joints):
joints.append(joint_name(i+1))
pub.append(rospy.Publisher(joints[i], Float64 , queue_size=10))




track= path_search(origin, destination, oc_grid,x2,y2) #A* search path as a list of coordinates


R= 0.08
L= 0.41
w_int= 0
old_e = 0
E = 0
w=0
for point in track:
    roll_x, pitch_y, yaw_z = euler_from_quaternion(xq, yq, zq, wq)
    des_x , des_y = point
    K_v=5
    K_d=10
    K_i= 3
    error= sqrt(pow(des_x-x,2)+pow(des_y-y,2)) #Proportional term
    e_dot= error - old_e #Differential term
    E = E + error #Integral term
    V= K_v*error + K_d*e_dot + K_i*E
    old_e = error

    theta_star= atan2(des_y-y,des_x-x)
    k_t= 3
    k_d= 5
    k_i= 2


    theta= yaw_z
    w_star= theta_star-theta
    w_dot= w_star-w
    w_int= w_int + w_star 
    W=k_t*w_star+ k_d*w_dot + k_i*w_int

    w= w_star

    Vl = V/R - (W*L)/(2*R)
    Vr = V/R + (W*L)/(2*R)

    pub[0].publish(Vl)
    pub[1].publish(-Vr)
    pub[2].publish(Vl)
    pub[3].publish(-Vr)
    rate.sleep()
