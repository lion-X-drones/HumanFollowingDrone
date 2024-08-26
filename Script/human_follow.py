#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from cvzone.PoseModule import PoseDetector



current_state = State()
vx =0
vy = 0
vz = 1.5
def state_cb(msg):
    global current_state
    current_state = msg

def wait_for(predicate, timeout):
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while not predicate() and rospy.Time.now() - start_time < rospy.Duration(timeout):
        rate.sleep()
    return rospy.Time.now() - start_time < rospy.Duration(timeout)

def arm_takeoff():
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    rospy.loginfo("Waiting for MAVROS connection...")
    wait_for(lambda: current_state.connected, 30.0)
    rospy.loginfo("MAVROS connected")

    rate = rospy.Rate(20.0)  # 20 Hz

    # Change mode to GUIDED
    rospy.loginfo("Changing mode to GUIDED")
    if not wait_for(lambda: current_state.mode == "GUIDED", 5.0):
        set_mode_client(custom_mode="GUIDED")

    # Arm the vehicle
    rospy.loginfo("Arming")
    if not wait_for(lambda: current_state.armed, 2.0):
        arming_client(True)
    
    for i in range(1,70):
    	rate.sleep()

    # Takeoff
    rospy.loginfo("Taking off")
    if takeoff_client(altitude=1.5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        rospy.loginfo("Takeoff sent")
    else:
        rospy.loginfo("Failed to send takeoff command")

class PoseControlNode:
    def __init__(self):
        #rospy.init_node('pose_control_node', anonymous=True)
        self.bridge = CvBridge()
        self.pose_detector = PoseDetector()
        self.vel_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
#        self.rate = rospy.Rate(10)  # Hz
        self.X = [537, 464, 341, 285, 236, 188, 153, 141]
        self.y = [50, 75, 100, 125, 150, 200, 250, 300]
        self.coff = np.polyfit(self.X, self.y, 2)

    def image_callback(self):
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        
        # except CvBridgeError as e:
        #     print(e)
        #     return
        cap = cv2.VideoCapture(3)
        global vx
        global vy
        global vz
        
        for i in range(1):
            _,cv_image = cap.read()
            cv_image= cv2.resize(cv_image, (768, 576))

            vel_msg = Point()
            
            stri=cv_image.dtype
            rospy.loginfo(f"{stri}")

            img = self.pose_detector.findPose(cv_image)
            lmList, bboxInfo = self.pose_detector.findPosition(img)
            # cv2.imshow("image",cv_image)
            cv2.imshow("image",cv_image)
            cv2.waitKey(1)

            if lmList:
                hand_x, hand_y = lmList[0][0], lmList[0][1]

                lrgap = 50
                lrvel = 1
                udgap = 35
                udvel = 1

                if hand_x < 340:
                    vx += -1 * ((340 - hand_x) // lrgap + 1) * lrvel
                elif hand_x > 420:
                    vx += ((hand_x - 420) // lrgap + 1) * lrvel

                if hand_y < 250:
                    vy += ((250 - hand_y) // udgap + 1) * udvel
                elif hand_y > 310:
                    vy += -1 * ((hand_y - 310) // udgap + 1) * udvel

            if bboxInfo:
                bbox = bboxInfo["bbox"]
                A, B, C = self.coff
                distanceCM = A * bbox[2] ** 2 + B * bbox[2] + C

                c = 25
                v = 1

                if distanceCM < 125:
                    vz += -1 * ((125 - distanceCM) // c+1) * v
                elif distanceCM > 175:
                    vz += ((distanceCM - 175) // c + 1) * v
            vx=round(vx,1)
            vy=round(vy,1)
            vz=round(vz,1)
            local=PoseStamped()
            rate=rospy.Rate(50)

            for i in range(50):
                vel_msg.x=vz
                vel_msg.y=vx
                vel_msg.z=1.5
                local.pose.position=vel_msg
                self.vel_pub.publish(local)
                rate.sleep()
            rospy.loginfo(f"[{vz} {vx} {vy}]")

    def run(self):
        
        #self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        
        self.image_callback()
        # while not rospy.is_shutdown():
            
        #     # rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
            
        #     self.rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node("human_node", anonymous=True)
    arm_takeoff()
    node = PoseControlNode()
    rate=rospy.Rate(1)
    for i in range(1,5):
        rate.sleep()
    while not rospy.is_shutdown():
        node.run()
        # rate.sleep()    
    




