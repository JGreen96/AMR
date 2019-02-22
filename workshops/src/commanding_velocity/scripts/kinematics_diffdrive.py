import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wheel_radius = 0.066
robot_radius = 0.287

# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a)
    c_r = v + (robot_radius * a)
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)
   
class Kinematics():
    
    def __init__(self):
        self.sub = rospy.Subscriber('/wheel_vel_left',
                                    Float32, self.callback)
                                    
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                       Twist, queue_size=1)
    
    def callback(self, data):
        twist_msg = Twist()
        
        (v, a) = forward_kinematics(data.data, 0.0)        
        
        print "v = %f,\ta = %f" % (v, a)
        
        
        twist_msg.linear.x = v
        twist_msg.angular.z = a
        
        self.cmd_vel_pub.publish(twist_msg)
        
        #r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        #for a in range(10):
        #    self.cmd_vel_pub.publish(twist_msg)
        #    r.sleep()
        
if __name__ == "__main__":

    rospy.init_node('kine')
    
    kine = Kinematics()
    rospy.spin()

#    (w_l, w_r) = inverse_kinematics(0.0, 1.0)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)
#
#    (v, a) = forward_kinematics(w_l, w_r)
#    print "v = %f,\ta = %f" % (v, a)
#
#    
#    t = Twist()
#
#    t.linear.x = 0.3
#    t.angular.z = 0.8
#
#    (w_l, w_r) = inverse_kinematics_from_twist(t)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)