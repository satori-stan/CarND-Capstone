import argparse
import cv2
import numpy as np
from keras import __version__ as keras_version
from keras.callbacks import ModelCheckpoint
from os import listdir

DATA_ROOT_PATH = '/tmp/data/tl_classification'

def load_image(base_path, image_path):
    """Reads an image file and returns a bgr8 array and its label

    Arguments:
    base_path -- The directory where the captures are saved
    image_path -- The name of the file to load
    """
    label = image_path.split('.')[0].split('-')[1]
    image = cv2.resize(cv2.imread(base_path + '/' + image_path), (224, 224)) # Assumes the file is a bgr8 encoded jpg
    #image = cv2.cvtColor(cv2.imread(base_path + '/' + image_path), cv2.COLOR_BGR2RGB)
    return image, label

def train():
    captures = os.listdir(DATA_ROOT_PATH)

    assert len(captures) > 0, "No files found!"

    from sklearn.model_selection import train_test_split
    train_samples, validation_samples = train_test_split(captures, test_size=0.2)

    import sklearn

    def batch_len(array):
        """Shortcut function to calculating the length of a batch from the
        generator based on the augmentations performed"""
        return len(array) * 2  # Mirror

    def generator(samples, batch_size=32):
        """Generator function to return a number of example instances for training

        Arguments:
        samples -- The full array of samples (X data and y result)
        batch_size -- The number of samples (before augmentation) that will be
                      returned by the generator. The samples are shuffled before
                      each new batch is generated.
        """
        def augment_and_append(image, label):
            images.append(image)
            labels.append(label)
            images.append(np.fliplr(image))
            labels.append(label)

        num_samples = len(samples)
        while 1:
            sklearn.utils.shuffle(samples)
            for offset in range(0, num_samples, batch_size):
                batch_samples = samples[offset:offset+batch_size]
                images = []
                labels = []
                for batch_sample in batch_samples:
                    image, label = load_image(DATA_ROOT_PATH, batch_sample)
                    augment_and_append(image, label)

                X_data = np.array(images)
                y_data = np.array(labels)
                yield sklearn.utils.shuffle(X_data, y_data)

    train_generator = generator(train_samples, batch_size=3)
    validation_generator = generator(validation_samples, batch_size=3)

    def _grouped_conv(X, is_stage_two=False, repeat=3):
        pass

    def _shuffle(X):
        pass

    def _shuffle_block(X, channels, is_stage_two=False, repeat=3):
        input_conv = None
        if is_stage_two:
            input_conv = Conv2D(channels, 1, 1, padding="same", activation="relu")(X)
        else:
            input_conv = _grouped_conv()(X)

        normalized_conv1 = BatchNormalization()(input_conv)
        shuffled = _shuffled(normalized_conv1)
        dw_3x3 = DepthwiseConv2D(3, 1, padding="same")(shuffled)
        normalized_conv2 = BatchNormalization()(dw_3x3)
        output_conv = _grouped_conv()(normalized_conv2)
        normalized_conv3 = BatchNormalization()(output_conv)
        added = Add()(X, normalized_conv3)
        return ReLu()(added)
        
    def _shuffle_block_stride2(X, channels, is_stage_two=False, repeat=3):
        input_conv = None
        if is_stage_two:
            input_conv = Conv2D(channels, 1, 1, padding="same", activation="relu")(X)
        else:
            input_conv = _grouped_conv()(X)

        normalized_conv1 = BatchNormalization()(input_conv)
        shuffled = ...
        #3x3 dw conv
        normalized_conv2 = BatchNormalization()(3x3_dw)
        output_conv = _grouped_conv()(normalized_conv2)
        normalized_conv3 = BatchNormalization()(output_conv)
        added = Add()(X, normalized_conv3)
        return ReLu()(added)

    def _shuffle_stage(X):
        pass

    # The original image size
    inputs = Input(shape=(600, 800, 3))
    # Now convert to a more convenient format
    #ncwh = tf.transpose(inputs, [0, 3, 1, 2])
    conv1 = Conv2D(24, 3, 2, padding="same", activation="relu")(inputs)
    pool1 = MaxPooling2D((2,2))(conv1)
    stage2 = _shuffle_stage(pool1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser
