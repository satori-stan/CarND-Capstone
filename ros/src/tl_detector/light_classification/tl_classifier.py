from keras.backend import tf as ktf
import numpy as np
from styx_msgs.msg import TrafficLight

from class_mapper import map_from
from model import load_model

class TLClassifier(object):
    def __init__(self):
        self.model = load_model('light_classification/M.h5')
        self.graph = ktf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #image = Image.open(BytesIO(base64.b64decode(imgString)))
        image_array = np.asarray(image)
        # XXX: Should we get the probability?
        with self.graph.as_default():
            prediction = self.model.predict(image_array[None, :, :, :], batch_size=1)
            # e.g. numpy.ndarray([[ 0.25199735  0.25194225  0.24826948  0.24779095]])
            prediction_list = prediction[0].tolist()
        return map_from(prediction_list.index(max(prediction_list)))
