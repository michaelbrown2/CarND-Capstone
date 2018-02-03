from styx_msgs.msg import TrafficLight
import rospy
import numpy as np
from keras.models import model_from_json
from keras import backend as K
import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy import misc
from time import time

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        K.clear_session()

        model_arch_path = 'models/model.json'
        model_weights_path = 'models/model_weights.h5'

        json_file = open(model_arch_path)
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        self.model._make_predict_function()

        self.model.load_weights(model_weights_path)
	#self.bridge = CvBridge()
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        resize_width, resize_height = (227, 227)
        resized_img = cv2.resize(image, (resize_width,resize_height), fx=0.38, fy=0.38)
        cropped_img = resized_img[0:227, 0:227]/255.0
        prediction = self.model.predict(np.array([cropped_img]))[0]
        prediction_labels = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.UNKNOWN, TrafficLight.YELLOW]
        labels_names = ['GREEN', 'RED', 'UNKNOWN','YELLOW']
        light_state = prediction_labels[prediction.argmax()]
        return light_state
