#!/usr/bin/env python
import rospy
import cv2
import os, sys
import numpy as np
import pandas as pd
import tensorflow as tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from numpy import inf
from pathlib import Path
from tensorflow.keras.models import load_model
np.set_printoptions(threshold=sys.maxsize)
global np_data, train_images_test_example, img_counter, camera_data
np_data = np.zeros((1440,))
img_counter = 0
model_distance = load_model('./autonomous_vehicle_regression.h5')
model_lidar = load_model('./lidar_DNN_plus.h5')
capture = cv2.VideoCapture(2)
    
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
def camera_data_setting():
    global train_images_test_example, train_images_test_example_length
    
    folder_directory_path_regression_test_example = './regression_test_example/'
    image_dir_test_example = Path(folder_directory_path_regression_test_example)
    
    filepaths_test_example = pd.Series(list(image_dir_test_example.glob(r'**/*.bmp')), name='Filepath_test_example').astype(str)
    length_test_example = pd.Series(filepaths_test_example.apply(lambda x: os.path.split(os.path.split(x)[0])[1]), 
                                                                             name='length_test_example').astype(np.int)
    images_test_example = pd.concat([filepaths_test_example, length_test_example], axis=1)
    
    train_generator_test_example = tf.keras.preprocessing.image.ImageDataGenerator(
        rescale=1./255
    )
    
    train_images_test_example = train_generator_test_example.flow_from_dataframe(
        dataframe=images_test_example,
        x_col='Filepath_test_example',
        y_col='length_test_example',
        target_size=(160,90),
        color_mode='grayscale',
        class_mode='raw',
        batch_size=32,
        shuffle=False
    )
    
def camera_regression(mirror=False):
    global img_counter, camera_data, angle
    camera_path = './regression_test_example/0/'
    
    ret, frame = capture.read()      # ret 은 True, False 반환, frame은 프레임
    if mirror:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   # grayscale로 변환
        
        img_name = "Opencv_camera_1_second.bmp".format(img_counter)
        cv2.imwrite(os.path.join(camera_path, img_name), gray)    #camera_path에 사진 저장
        camera_data = model_distance.predict(train_images_test_example, verbose = 0)
        
        img_counter += 1
    
def lidar_DNN():
    global np_data, bowl, angle
    
    bowl = []
    # LIDAR data setting about infinite / people consideration
    np_data[np_data == inf] = 0
    np_data[520:919] = 0
    
    # dual list numpy array create
    bowl.append(np_data)
    bowl = np.asarray(bowl)
    
    # DNN Model Prediction (result : angle [0~8])
    angle = model_lidar.predict_classes(bowl)
    
def emergency_brake():
    global bowl, test_data, idx_list, length, brake_point
    
    # -20 ~ +20 degree range = emergency brake range
    right_data = bowl[0, 0:80]
    left_data = bowl[0, 1360:1439]
    test_data = np.concatenate((right_data, left_data), axis = 0)
    
    idx_list = np.where((test_data >= 0.2) & (test_data <= 2.0))
    idx_list = np.asarray(idx_list)
    length = idx_list.shape
    
    if (length[1] >= 20):
        brake_point = "O"
    else:
        brake_point = "X"
        
def callback(data):
    global np_data
    np_data = np.array(data.ranges)
    
def talker():
    global np_data, img_counter, train_images_test_example, camera_data, angle
    
    # ROS Node Setting (한번 설정하면 바꿀 수 없음)
    rospy.init_node('talker', anonymous=True)
    
    # Publisher Setting (name : 'chatter', type : 'String')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # LIDAR Sensor Data Collection Setting
        rospy.Subscriber("/scan", LaserScan, callback)
        
        # LIDAR Sensor Data -> numpy array transform
        lidar_DNN()
        
        # Camera CNN Model
        camera_startup()
        
        # Emergency Brake
        emergency_brake()
        
        # talker -> listener communication
        trans_data = "%s, %s, %f" % (angle, brake_point, camera_data)
        
        # loginfo() == ros식 print() function
        rospy.loginfo("%s" % trans_data)
        
        # Talker -> Listener Data Communication
        pub.publish(trans_data)
        rate.sleep()
        
    rospy.spin()
def camera_startup():
    camera_regression(mirror=True)
if __name__ == '__main__':
    try:
        camera_data_setting()
        talker()
    except rospy.ROSInterruptException:
        pass