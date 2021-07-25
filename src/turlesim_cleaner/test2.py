import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


x=0
y=0
z=0
yaw=0


def poseCallback(message):
    global x
    global y, z, yaw
    x= message.x
    y= message.y
    yaw = message.theta
    print(message)


rospy.init_node('turtlesim_motion_pose', anonymous=True)

#declare velocity publisher
        
velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        
        
pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback) 
rospy.spin()










    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move():
        #declare a Twist message to send velocity commands
            twist = Twist()
            #get current location 
            
            #z0=z;
            #yaw0=yaw;
            twist.linear.x = 1.0
            twist.angular.z = 1.0

            
            loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
            
            velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

            while not rospy.is_shutdown():
                    rospy.loginfo("Turtlesim moves forwards")
                    velocity_publisher.publish(twist)

                    loop_rate.sleep()
                    
                    #rospy.Duration(1.0)
                    
                                 
                    
            
            #finally, stop the robot when the distance is moved
            
    


if __name__ == '__main__':
    try:
        move ()
        
        
        
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")