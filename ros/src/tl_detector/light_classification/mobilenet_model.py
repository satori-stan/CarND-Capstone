#!/bin/env python

import argparse
import math
import os

import cv2
from keras.callbacks import ModelCheckpoint
import numpy as np

from class_mapper import map_to
from model import load_model

# IMPORTANT: Keep the slash at the end!
DATA_ROOT_PATH = '/tmp/data/tl_classification/'

def load_image(base_path, image_path):
    """Reads an image file and returns a bgr8 array and its label

    Arguments:
    base_path -- The directory where the captures are saved
    image_path -- The name of the file to load
    """
    full_path = os.path.join(base_path, image_path)
    label = map_to(int(image_path.split('/')[0]))
    image = cv2.imread(full_path) # Assumes the file is a bgr8 encoded jpg
    #image = cv2.cvtColor(cv2.imread(base_path + '/' + image_path), cv2.COLOR_BGR2RGB)
    return image, label

def train(weights_file):
    captures = []  #os.listdir(DATA_ROOT_PATH)

    for dirpath, subdirs, files in os.walk(DATA_ROOT_PATH):
        if len(files) > 0:
            for f in files:
                captures.append(os.path.join(dirpath, f).replace(DATA_ROOT_PATH, ''))

    assert len(captures) > 0, "No files found!"

    from sklearn.cross_validation import train_test_split
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

                #print(labels)
                X_data = np.array(images)
                y_data = np.array(labels)
                yield sklearn.utils.shuffle(X_data, y_data)

    train_generator = generator(train_samples, batch_size=10)
    validation_generator = generator(validation_samples, batch_size=10)

    #inputs = Input(shape=(600, 800, 3))
    #resized = Lambda(lambda image: ktf.image.resize_images(image, (224, 224)))(inputs)
    #model = MobileNet(alpha=2, depth_multiplier=1, include_top=True, weights=None, classes=4, input_tensor=resized)

    #model.compile(loss='mse', optimizer='adam')

    #if not base_model is None:
    #    model.load_weights(base_model)

    model = load_model(weights_file)
    checkpoint = ModelCheckpoint('M_{val_loss:.4f}.h5', monitor='val_loss', verbose=1, save_best_only=True, mode='min', save_weights_only=True)
    callbacks_list = [checkpoint]

    history = model.fit_generator(train_generator, steps_per_epoch=2000, epochs=30, callbacks=callbacks_list, validation_data=validation_generator, validation_steps=30, use_multiprocessing=True)

    model.save('M.h5')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Traffic Light Classification Training")
    parser.add_argument(
        'model_weights',
        type=str,
        help="Path to a weights file that will be fine-tuned"
    )
    args = parser.parse_args()
    train(args.model_weights)
