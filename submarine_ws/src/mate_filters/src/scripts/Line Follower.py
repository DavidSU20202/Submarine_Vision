import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ros_numpy
import cv2
import numpy as np
from csv import unregister_dialect

ROTOPIC_NAME="/robocol/vision/cam_0"
Dirr = True
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (255, 0, 0)
thickness = 2
rsize=90
cint = None
crect = None
drect = None
urect = None
lrect = None
rrect = None
dim = None
CutImg=[1,1,1,1,1]
cond=[1,1,1,1,1]

def existred(frame):
    return 

def IsRed(img):
    if ((img>0).mean()>=0.1):
        return True
    return False

def calibrateParameters(frame):
    global cint
    global crect
    global drect
    global urect
    global lrect 
    global rrect 
    global dim

    height = frame.shape[0]
    width = frame.shape[1]
    c = (width/2,height/2)
    cint = (int(width/2),int(height/2))
    crect = ((int(c[0]-(rsize/2)),int(c[1]+(rsize/2))),(int(c[0]+(rsize/2)),int(c[1]-(rsize/2))))
    drect = ((int(crect[0][0]),int(crect[0][1]+rsize)),(int(crect[1][0]),int(crect[1][1]+rsize)))
    urect = ((int(crect[0][0]),int(crect[0][1]-rsize)),(int(crect[1][0]),int(crect[1][1]-rsize)))
    lrect = ((int(crect[0][0]-rsize),int(crect[0][1])),(int(crect[1][0]-rsize),int(crect[1][1])))
    rrect = ((int(crect[0][0]+rsize),int(crect[0][1])),(int(crect[1][0]+rsize),int(crect[1][1])))
    scale_percent = 50 # percent of original size
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)


def image_recived(msg):
    global cint

    # if MOVEMENT_RECIVED:
    try:
        frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
    except:
        frame = ros_numpy.numpify(msg)
    if cint is None:
        calibrateParameters(frame)
    filterImage(frame)
    
def filterImage(frame):
    global Dirr
    global cond
    global CutImg
    global cint


    img1=frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Our operations on tlower = np.array([155,25,0])
    #lower red

    lower_red = np.array([0,170,170])
    upper_red = np.array([10,255,255])


    #upper red
    lower_red2 = np.array([170,50,50])
    upper_red2 = np.array([180,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(img1,img1, mask= mask)


    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    res2 = cv2.bitwise_and(img1,img1, mask= mask2)

    img3 = res+res2

    # Display the resulting frame
    
    CutImg[0]= img3[crect[1][1]:crect[0][1], crect[0][0]:crect[1][0]] #crectimg 
    CutImg[1]= img3[drect[1][1]:drect[0][1], drect[0][0]:drect[1][0]] #drectimg 
    CutImg[2]= img3[urect[1][1]:urect[0][1], urect[0][0]:urect[1][0]] #urectimg 
    CutImg[3]= img3[lrect[1][1]:lrect[0][1], lrect[0][0]:lrect[1][0]] #lrectimg 
    CutImg[4]= img3[rrect[1][1]:rrect[0][1], rrect[0][0]:rrect[1][0]] #rrectimg 
    for i in range(0,len(CutImg)):
        cond[i]=IsRed(CutImg[i])

    if cond[0]:
        if cond[2] and cond[1]:
            frame = cv2.putText(frame, 'Down', cint, font, fontScale, color, thickness, cv2.LINE_AA)
        elif cond[1] and (cond[3] or cond[4]):
            frame = cv2.putText(frame, 'Down', cint, font, fontScale, color, thickness, cv2.LINE_AA)          
        elif cond[2]:
            Dirr=not(Dirr)
            if cond[3]:
                frame = cv2.putText(frame, 'Left', cint, font, fontScale, color, thickness, cv2.LINE_AA)
            elif cond[4]:
                frame = cv2.putText(frame, 'Right', cint, font, fontScale, color, thickness, cv2.LINE_AA)
        elif cond[4] and cond[3]:
            if Dirr:
                frame = cv2.putText(frame, 'Right', cint, font, fontScale, color, thickness, cv2.LINE_AA)
            else:
                frame = cv2.putText(frame, 'Left', cint, font, fontScale, color, thickness, cv2.LINE_AA)
    print(cond)
    cv2.imshow('res3',img3)
    cv2.imshow("Line Follower", frame)
    cv2.waitKey(5) 
        


if __name__ == '__main__':
    rospy.init_node('Line_Follower', anonymous=True)
    rospy.loginfo("Hello ROS!")
    sub_image = rospy.Subscriber(ROTOPIC_NAME, Image, image_recived)
    while not rospy.is_shutdown():
        rospy.spin()