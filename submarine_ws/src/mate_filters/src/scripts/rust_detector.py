import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ros_numpy as np
import cv2

ROTOPIC_NAME="/robocol/vision/cam_0"
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
thickness              = 1
lineType               = 2
dim=None

def image_recived(msg):
    print("[INFO]: Image Received, showImage function called")
    # if MOVEMENT_RECIVED:

    frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg)

    if dim is None:
        calibrateParameters(frame)
    
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    cv2.imshow("Rust Detector", frame)
 
    cv2.waitKey(1)
    #filterImage(frame,50,90,250)
  
def calibrateParameters(img):
    global dim 
    scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)  
    dim = (width, height)

# This function allows us to create a descending sorted list of contour areas.
def contour_area(contours):
    cnt_area = []
 
    for i in range(0,len(contours),1):
        cnt_area.append(cv2.contourArea(contours[i]))
 
    list.sort(cnt_area, reverse=True)
    return cnt_area

def draw_bounding_boxes(contours, image):
    cnt_area = contour_area(contours)
 
    for i in range(0,len(contours),1):
        cnt = contours[i]
        x,y,w,h = cv2.boundingRect(cnt)         
        image=cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
 
    return image


def rust_detector(img):
        
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower = np.array([10, 100, 20])
    upper = np.array([20, 255, 200])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)

    contours,hierarchy = cv2.findContours(mask, 1, 2)
    bounding = draw_bounding_boxes(contours, img)
        
        
    cv2.imshow("frame", img)


if __name__ == '__main__':
    rospy.init_node('Hole_Detection', anonymous=True)
    rospy.loginfo("Hello ROS!")
    sub_image = rospy.Subscriber(ROTOPIC_NAME, Image, image_recived)
    while not rospy.is_shutdown():
        rospy.spin()