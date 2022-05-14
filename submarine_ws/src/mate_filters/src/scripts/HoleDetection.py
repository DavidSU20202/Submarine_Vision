import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ros_numpy
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
    cv2.imshow("Hole Detection", frame)
 
    cv2.waitKey(1)
    #filterImage(frame,50,90,250)
  
def calibrateParameters(img):
    global dim 
    scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)  
    dim = (width, height) 

def filterImage(img,min_canny,max_canny,contArea):
    
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    conts = 0

    edges = cv2.Canny(img,min_canny, max_canny)

    contours, hierarchy= cv2.findContours(edges,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    for x in contours:
        if cv2.contourArea(x) > contArea:
            cv2.drawContours(img, x, -1, (0,255,0), 3)
            conts += 1


    cv2.putText(img,str(conts), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
    cv2.imshow("Hole Detection", img)
    print(f"Hoyos encontrados: {conts}")
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('Hole_Detection', anonymous=True)
    rospy.loginfo("Hello ROS!")
    sub_image = rospy.Subscriber(ROTOPIC_NAME, Image, image_recived)
    while not rospy.is_shutdown():
        rospy.spin()