#!/usr/bin/env python
# coding: utf-8

# In[1]:


import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


# In[3]:


import numpy as np
import chardet
from numpy import array
from numpy import uint8
import base64
import time


#test = str(img)
#print(test)
#test2 = np.fromstring(test,dtype=np.uint8)
#print(test2)
#test2.shape = 720,1280,3
#print(type(test2))
#print(type(img))
#print(img.shape)

#starting_time = time.time()
#img_bytes = img.tostring()
#with open('/home/reg1s/Postgraduate/TeamProject/model/Tiny-DSOD/Tiny-DSOD/vis/voc/frame4053.jpg', 'rb') as f:
 #   b64_code = base64.b64encode(f.read())

#b64_code = base64.b64encode(img_bytes)
#img_string = str(b64_code,encoding='utf-8')
#elapsed_time = time.time() - starting_time
#print(chardet.detect(img_bytes))
#img_string =img_bytes.decode('base64')

#img_bytes2 = bytes(img_string,encoding = "utf-8")
#b64_code2 = bytes(img_string,encoding='utf-8')
#bytes_decode = base64.b64decode(b64_code2)
#img3 = np.frombuffer(bytes_decode, np.uint8)
#img3.shape = 720,1280,3
#img2 = np.frombuffer(img_bytes,dtype=np.uint8)
#img2.shape = 720,1280,3
        
#img_str = cv2.imencode('.jpg', img)[1].tostring()
#print(type(img_str))    
    
#cv2.imshow('img',img3)
#cv2.waitKey(0)
#cv2.imwrite('/home/reg1s/test.jpg',np.asarray(img_list))


# In[13]:


sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from std_msgs.msg import String
from beginner_tutorials.msg import BBox

IMAGE_WIDTH=1280
IMAGE_HEIGHT=720

def publish_image(imgdata,image_publish):
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height=IMAGE_HEIGHT
    image_temp.width=IMAGE_WIDTH
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tostring()
    #print(imgdata)
    #image_temp.is_bigendian=True
    image_temp.header=header
    image_temp.step=1280*3
    image_publish.publish(image_temp)

#pub = rospy.Publisher('Image_string', String, queue_size=10)
pub_img = rospy.Publisher('Image', Image, queue_size=10)
pub_bbox = rospy.Publisher('BBox', BBox, queue_size=10)
rospy.init_node('image_converter', anonymous=True)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    img = cv2.imread('/home/reg1s/Postgraduate/TeamProject/model/Tiny-DSOD/Tiny-DSOD/vis/voc/frame4053.jpg')
    #pub.publish(img_string)
    bbox_data = BBox();
    bbox_data.x = 1;
    bbox_data.y = 2;
    bbox_data.h = 3;
    bbox_data.w = 4;
    publish_image(img,pub_img)
    pub_bbox.publish(bbox_data)
    rate.sleep()
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', 'fine')
    #f = open('/home/reg1s/catkin_ws/src/beginner_tutorials/scripts/base64.txt','w')
    #f.write(img_string)
    #f.close()

# In[ ]:




