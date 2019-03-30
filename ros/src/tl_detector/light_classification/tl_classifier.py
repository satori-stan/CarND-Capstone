from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #image = Image.open(BytesIO(base64.b64decode(imgString)))
        #image_array = np.asarray(image)
        #model.predict(image_array[None, :, :, :], batch_size=1)
        return TrafficLight.UNKNOWN
