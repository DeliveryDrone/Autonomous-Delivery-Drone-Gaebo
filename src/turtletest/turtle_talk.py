import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def callback(message):
    
    
    print(message)
    
def listener():
    

    rospy.Subscriber("/turtle1/pose", Pose, callback)
    rospy.spin()

def talker():
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size= 10)

    

    rate = rospy.Rate(1)


    while True:
        twist = Twist()
        twist.linear.x = 0.5
        twist.linear.y = 0.0
        twist.angular.z = 1.0
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()  


if __name__ == "__main__":
    try:
        rospy.init_node("listener", anonymous=True)
        
        talker()
        listener()
        
        
    except rospy.ROSInterruptException:
        pass