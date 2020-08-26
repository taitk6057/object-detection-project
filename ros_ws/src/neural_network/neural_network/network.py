import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import os

# for publisher
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import keras

# retinanet
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color

np_image_global = []
exit_signal = False
new_data = False
class_names = {}

class NN(Node): # class to subclass ros node (#1)

    def __init__(self, model_path, **kwargs):
        self.publisher_ = self.create_publisher(String, '/predictions', 1)
        
        # Load weights and set parameters (#2)
        self.model = load_model(model_path, backbone_name='resnet50')

        self.n_classes = 1
        self.image_shape = (224,224) # resnet50 dim
        self.confidence_thresh = 0.3 #copied from inference engine

        # run
        self.use_mono = kwargs.get('useMono', True)
        self.run()

    # Create subscription (#3)
    def callback(image_msg):
        # use cv bridge to convert image from ros type
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        cv_image = cv2.resize(cv_image, target_size)  # resize image for resnet50
        np_image = np.asarray(cv_image)               # read opencv img as np array
        np_image = np.expand_dims(np_image, axis=0)   # add another dimension for tensorflow (expand array)
        np_image = np_image.astype(float)  # convert data from uint8 to float64
        np_image = preprocess_input(np_image)         # normalize data

    # Publish detections (#5)
    def run_mono(self):
        #process images
        global class_names
        global np_image_global, new_data, exit_signal

        while not exit_signal:
            if new_data:
                np_image = np.copy(np_image_global)
                new_data = False
                boxes, scores, classes = self.model.predict_on_batch(np_image)
                time_found = time.time()
                class_labels = [class_names[c] for c in classes[0] if c != -1]
                boxes = [b for b,c in zip(boxes[0], classes[0]) if c != -1]
                scores = [s for s,c in zip(scores[0], classes[0]) if c != -1]
                for box, score, c_name in zip(boxes, scores, class_labels):
                    if score > self.confidence_thresh:
                        msg = String()
                        x1,y1,x2,y2 = box
                        # Convert from np -> python float to make ROS happy
                        msg.x1 = float(x1)
                        msg.x2 = float(x2)
                        msg.y1 = float(y1)
                        msg.y2 = float(y2)
                        msg.class_name = c_name
                        msg.confidence = int(round(score*100))
                        msg.image_name = 'temp'
                        # format(class, score, x1,y1,x2,y2)'
                        msg.data = 'Detected a {} with confidence {} at coordinates {}, {}, {}, {}'.format(class_labels,scores,x1,y1,x2,y2)
                        self.publisher_.publish(msg)
                        self.get_logger().info('Publishing detection:')

            else:
                time.sleep(0.01) #wait in secs copied from inference
        
         # Shutdown
        exit_signal = True

    def run(self):
        if self.use_mono:
            self.run_mono()

def main(args=None):
    rclpy.init(args=args)
    global class_names
    neural_network = NN('/home/taitk/Desktop/cyan/onboarding/ros_ws/neural_network/neural_network/models/weights/retinanet.h5')
    class_names = class_to_labels('/home/taitk/Desktop/cyan/onboarding/ros_ws/neural_network/neural_network/models/classes/coco.names', retinanet=True)
    rclpy.spin(NN)
    NN.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
