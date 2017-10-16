from styx_msgs.msg import TrafficLight
import rospy

import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from utilities import label_map_util
from utilities import visualization_utils as vis_util
import time

class TLClassifier(object):
    def __init__(self):
        #self.simulator = True #Set to false for real car.
        self.simulator = rospy.get_param("/simulator") #Set to false for real car.
      
        cwd = os.path.dirname(os.path.realpath(__file__)) 
        if self.simulator == True:
            print("In SIMULATOR")
            CKPT = cwd + "/graphs/frozen_inference_graph.pb"
        else:
            print("In CAR")
            CKPT = cwd + "/graphs/frozen_inference_graph_real.pb"

        PATH_TO_LABELS = cwd+'/graphs/object-detection.pbtxt'
        NUM_CLASSES = 3 # 3, as pretrained on udacity real car dataset with 3 classes

        #Label maps map indices to category names
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        #Build network
        self.detection_graph = tf.Graph()
        
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        print("Loaded model")
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        run_network = True  # flag to disable running network if desired
        if run_network is True:
            image_np_expanded = np.expand_dims(image, axis=0)

            time0 = time.time()

            # Actual detection.
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores,
                    self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})

            time1 = time.time()

            #print("Time in milliseconds", (time1 - time0) * 1000)

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
     
            #Add extra stuff here
            self.current_light = TrafficLight.UNKNOWN
            min_score_thresh = .50
            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > min_score_thresh:

                    class_name = self.category_index[classes[i]]['name']
                    # class_id = self.category_index[classes[i]]['id']  # if needed

                    #print('{}'.format(class_name))

                    # Traffic light thing
                    self.current_light = TrafficLight.UNKNOWN

                    if class_name == 'Red':
                        self.current_light = TrafficLight.RED
                    elif class_name == 'Green':
                        self.current_light = TrafficLight.GREEN
                    elif class_name == 'Yellow':
                        self.current_light = TrafficLight.YELLOW
            
            '''if self.current_light == TrafficLight.RED:
                print('RED')
            elif self.current_light == TrafficLight.GREEN:
                print('GREEN')
            elif self.current_light == TrafficLight.YELLOW:
                print('YELLOW')
            else:
                print('NO_LIGHT')'''

        return self.current_light  
