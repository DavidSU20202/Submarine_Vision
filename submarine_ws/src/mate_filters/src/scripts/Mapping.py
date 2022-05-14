import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ros_numpy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.feature import hog
import skimage.filters as fl
from skimage.morphology import dilation, disk
import pickle as pkl

ROTOPIC_NAME="/robocol/vision/cam_0"
ventana=[100,960]
fig,ax=plt.subplots(2)
fig2,ax2=plt.subplots(8)
s=True
s1=0
S=0
fr=0
a=np.ones((1088,1920,3),dtype=np.uint8)*112
a2=np.ones((1088,1920,3),dtype=np.uint8)*112

with open('tr_1.mat','rb') as f:
    seg1=pkl.load(f)
with open('tr_2.mat','rb') as f:
    seg2=pkl.load(f)
with open('tr_3.mat','rb') as f:
    seg3=pkl.load(f)

with open("modeloLinea.dat", "rb") as f:
    modeloLinea=pkl.load(f)

def segmentacion(Im):
    from skimage import segmentation, color
    from skimage.future import graph
    import time

    labels1 = segmentation.slic(Im, compactness=0.8, n_segments=500,
                                start_label=1)
    g = graph.rag_mean_color(Im, labels1, mode='similarity')
    labels2 = graph.cut_normalized(labels1, g)
    out2 = color.label2rgb(labels2, Im, kind='avg', bg_label=0)
    t2=time.time()
    return out2

def image_recived(msg):
    print("[INFO]: Image Received, showImage function called")
    # if MOVEMENT_RECIVED:

    frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
 
    cv2.waitKey(1)
    filterImage(frame,False)

def umbral(Linea):
    Linea_0 = Linea[:, :, 0]
    Linea1_0 = Linea_0 < 65
    Linea3_0 = Linea_0 > 30

    Linea_1 = Linea[:, :, 1]
    Linea1_1 = Linea_1 < 60
    Linea3_1 = Linea_1 > 40

    Linea_2 = Linea[:, :, 2]
    Linea1_2 = Linea_2 < 75
    Linea3_2 = Linea_2 > 40

    Linea0 = Linea1_0 * Linea3_0
    Linea0 = fl.gaussian(Linea0)
    Linea0 = dilation(Linea0, disk(3))

    Linea1 = Linea1_1 * Linea3_1
    Linea1 = fl.gaussian(Linea1)
    Linea1 = dilation(Linea1, disk(3))

    Linea2 = Linea1_2 * Linea3_2
    Linea2 = fl.gaussian(Linea2)
    Linea2 = dilation(Linea2, disk(3))

    Linea = Linea0 * Linea1 * Linea2
    return Linea

def Line(l):
    x,y=l.shape
    lz=np.zeros((x,y,3))
    lz[:,:,0]=l
    return lz*255

def Descriptor(Linea,mode='train'):
    Linea = cv2.cvtColor(Linea, cv2.COLOR_BGR2HSV)

    Linea=umbral(Linea)
    analisis=True
    if mode=='val':
        if np.sum(Linea)<1000:
            analisis=False
        if analisis:
            fd, Hog_Linea = hog(Linea, visualize=True, orientations=40, pixels_per_cell=(10, 96))
        else:
            fd=0
        return fd, analisis
    else:
        fd, Hog_Linea = hog(Linea, visualize=True, orientations=40, pixels_per_cell=(10, 96))
        return fd

def deteccion(I,ventana):
    x, y, z = I.shape
    X = np.arange(300, 600, 150)
    Y = np.arange(0, y, 150)
    Image=np.copy(I)
    count=0
    count2=0
    L=[]
    cont=True
    for i in X:
        for j in Y:
            if cont:
                if i + ventana[0] < x and j + ventana[1] < y:
                    r = I[i:i + ventana[0], j:j + ventana[1]]
                    des=Descriptor(r,'val')
                    if des[1]:
                        count+=1
                        pred=modeloLinea.predict([des[0]])
                        if pred[0]==1:
                            count2+=1
                            L=umbral(I)
                            Image = cv2.rectangle(Image, (j, i),
                                                  (j + ventana[1], i + ventana[0]), (255, 0, 0))
                            cont=False
                    else:
                        continue
    return Image,count2,L


def filterImage(img,tram):
    global s
    global s1
    global S
    global seg1
    global seg2
    global seg3
    global a
    global a2
    global fr

    if fr%2==0:
        detec = deteccion(img, ventana)
        # cv2.imshow('output',detec[0])
        ax[0].cla()
        ax[0].imshow(cv2.cvtColor(detec[0], cv2.COLOR_BGR2RGB))
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 2]
        if s == False:
            s1 += 1
        if s1 == 20:
            s1 = 0
            s = True
        if detec[1] >= 1 and s:
            S += 1
            ax[1].cla()
            # A = segmentacion(frame)
            if tram:
                if S == 1:
                    A = seg1
                if S == 2:
                    A = seg2
                if S == 3:
                    A = seg3
                    A2 = np.copy(A)
                    for i in range(len(A[0])):
                        A2[:, i] = A[:, (i - 206) % len(A[0])]
                    A = A2
            else:
                A = segmentacion(frame)
            ax2[(S - 1) * 2].imshow(A[:, :int(len(A[0]) / 2), :])
            ax2[(S - 1) * 2 + 1].imshow(A[:, int(len(A[0]) / 2):, :])
            laterales = np.zeros((len(A), 40, 3))
            laterales[:, :, 0] = np.ones((len(A), 40)) * 255
            A[:, :40, :] = laterales
            A[:, -40:, :] = laterales
            A[:, int(len(A[0]) / 2) - 20:int(len(A[0]) / 2) + 20, :] = laterales
            horizontales = np.zeros((10, len(A[0]), 3))
            horizontales[:, :, 0] = np.ones((10, len(A[0]))) * 255
            A[:10, :, :] = horizontales
            A[-10:, :, :] = horizontales
            A = np.array(A, dtype=np.uint8)
            a = np.concatenate((A, a), axis=0)
            if S == 3:
                a = np.concatenate((a2, a), axis=0)
            ax[1].imshow(a)
            s = False
        plt.pause(0.5)
    fr+=1
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('Mappingg', anonymous=True)
    rospy.loginfo("Hello ROS!")
    sub_image = rospy.Subscriber(ROTOPIC_NAME, Image, image_recived)
    while not rospy.is_shutdown():
        rospy.spin()