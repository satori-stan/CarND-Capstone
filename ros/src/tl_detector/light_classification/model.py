from keras.applications.mobilenet import MobileNet
from keras.backend import tf as ktf
from keras.layers import Input,Lambda

def load_model(weights_file):
    inputs = Input(shape=(600, 800, 3))
    resized = Lambda(lambda image: ktf.image.resize_images(image, (224, 224)))(inputs)
    model = MobileNet(alpha=2, depth_multiplier=1, include_top=True, weights=None, classes=4, input_tensor=resized)

    model.compile(loss='mse', optimizer='adam')

    if not weights_file is None:
        model.load_weights(weights_file)

    return model

