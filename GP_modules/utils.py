from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier  # MLP is an NN
from sklearn import svm
import numpy as np
import argparse
import imutils  
import cv2  
import os
import random
import matplotlib.pyplot as plt
from skimage.feature import hog
from skimage import data, exposure
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
def preprocess_image_fromPath(image_path, target_size=(64, 64)):
    """
    Preprocess the image by resizing, grayscaling, applying noise reduction, and normalizing.
    Args:
        image_path: Path to the image.
        target_size: Target size for resizing.
    Returns:
        Preprocessed image.
    """
    
    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Image not found at {image_path}")
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply a Median filter for further noise reduction (optional)
    filtered = cv2.medianBlur(blurred, 3)
    
    # Resize to the target size
    resized = cv2.resize(filtered, target_size)
    
    # Normalize pixel values to [0, 1]
    normalized = resized / 255.0
    
    return normalized



def extract_hog_features(image, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2)):
    """
    Extract HOG features from the image.
    Args:
        image: Preprocessed image (grayscale).
        orientations: Number of orientation bins.
        pixels_per_cell: Size of the cell in pixels.
        cells_per_block: Number of cells in each block.
    Returns:
        HOG feature vector.
    """


    hog_features = hog(image, orientations=orientations,
                       pixels_per_cell=pixels_per_cell,
                       cells_per_block=cells_per_block,
                       block_norm='L2-Hys', visualize=False)
    '''
hog(image, ...): This function, imported from skimage.feature, computes the HOG features for the input image based on the specified parameters.

orientations=orientations: Specifies the number of bins for the gradient histogram.

pixels_per_cell=pixels_per_cell: Defines the dimensions of the cells for which the gradient histogram is computed.

cells_per_block=cells_per_block: Specifies the size of the block used for local contrast normalization.

block_norm='L2-Hys': Indicates the normalization method to use for the blocks. L2-Hys is a widely used normalization technique that improves robustness to lighting and contrast changes.

visualize=False: Disables the visualization of the HOG image, which is useful for debugging but not required for feature extraction.
'''
    return hog_features


def predict_image_fromPath(image_path, model, class_mapping, target_size=(64, 64)):
    """
    Predict the class of a new image.
    Args:
        image_path: Path to the image.
        model: Trained SVM model.
        class_mapping: Mapping of class names to labels.
        target_size: Target size for resizing.
    Returns:
        Predicted class name.
    """
    # Preprocess and extract features
    preprocessed_image = preprocess_image_fromPath(image_path, target_size)
    feature_vector = extract_hog_features(preprocessed_image).reshape(1, -1)
    # Predict
    prediction = model.predict(feature_vector)[0]
    # Map label back to class name
    for class_name, label in class_mapping.items():
        if label == prediction:
            return class_name
