import rospy
from ros_control_single_motor.msg import JointStateMsg
from ros_control_single_motor.srv import Float_Floats, Float_FloatsResponse


pos = 0
vel = 0
def cb_from_arduino(msg: JointStateMsg):
    global pos, vel
    pos = msg.pos
    vel = msg.vel
    # rospy.loginfo(f"pos:{msg.pos}, vel:{msg.vel}")
    
def pos_and_vel_server(req):
    global pos, vel
    res = Float_FloatsResponse()
    res.res = [pos, vel]
    return res

def main():
    rospy.init_node('arduino_agent')
    rospy.Subscriber('/joint_state_from_arduino', JointStateMsg, cb_from_arduino, queue_size=10)
    rospy.Service('/read_joint_state', Float_Floats, pos_and_vel_server)
    rospy.loginfo(f"Listening")
    rospy.spin()

if __name__ == '__main__':
    try:
        # print("Hello")
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass