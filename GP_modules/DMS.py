'''
50 % of this code was written by LLM for interfacing with carla simulator
'''
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
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
import time
import pickle
import dlib
import threading
import queue

class DMSModule:
    def __init__(self):
        """
        Service Name: DMSModule
        Sync/Async: Asynchronous
        Reentrancy: Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: None
        Description: Initializes the DMS module with threading support for async processing.
        """
        self.CLOSED_EYE_THRESHOLD = 3 
        self.SIDE_FACE_THRESHOLD = 3 
        self.OPEN_EYE_THRESHOLD = 1  
        self.SIDE_FACE_ANGLE_THRESHOLD = 30  
        self.closed_eye_start_time = None
        self.closed_eye_duration = 0
        self.open_eye_start_time = None
        self.open_eye_duration = 0
        self.side_face_start_time = None
        self.side_face_duration = 0
        predictor_file = "GP_modules/shape_predictor_68_face_landmarks.dat"
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(predictor_file)
        self.cap = cv2.VideoCapture(0)
        archive_dir = "archive"
        if not os.path.exists(archive_dir):
            os.makedirs(archive_dir)
            print(f"Created directory: {archive_dir}")
        self.kalman_filter = self._create_kalman_filter()
        try:
            self.class_mapping = self._load_class_mapping()
            self.model = self._load_model()
        except Exception as e:
            print(f"Error loading models: {e}")
            self.class_mapping = {}
            self.model = None
        self._status_lock = threading.Lock()
        self.is_driver_sleeping = False
        self.is_driver_distracted = False
        self.frame_queue = queue.Queue(maxsize=2) 
        self.result_queue = queue.Queue(maxsize=2)
        self.thread_active = True
        self.processing_thread = threading.Thread(target=self._processing_thread_function, daemon=True)
        self.processing_thread.start()
        self.frame_counter = 0
        self.process_every_n_frames = 2 
        self.latest_frame = None
        self.last_update_time = 0
        self.update_timeout = 5.0  
        
        print("DMS Module initialized with threading support")

    def _processing_thread_function(self):
        """
        Service Name: _processing_thread_function
        Sync/Async: Asynchronous
        Reentrancy: Non-Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: None
        Description: create thread that processes camera frames for DMS analysis.
        """
        while self.thread_active:
            try:
                ret, img = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue
            
                sleeping, distracted, processed_frame = self._process_frame_internal(img)
                with self._status_lock:
                    self.is_driver_sleeping = sleeping
                    self.is_driver_distracted = distracted
                    self.last_update_time = time.time()
                try:
                    self.result_queue.put(processed_frame, block=False)
                except queue.Full:
                    try:
                        self.result_queue.get_nowait()
                        self.result_queue.put(processed_frame, block=False)
                    except queue.Empty:
                        pass
                time.sleep(0.05)  
                
            except Exception as e:
                print(f"Error in DMS processing thread: {e}")
                time.sleep(0.1)
        
        print("DMS processing thread terminated")

    def _process_frame_internal(self, img):
        """
        Service Name: _process_frame_internal
        Sync/Async: Synchronous (but called from async thread)
        Reentrancy: Non-Reentrant
        Parameters (in): img - Camera frame
        Parameters (inout): None
        Parameters (out): None
        Return value: (is_sleeping, is_distracted, processed_frame)
        Description: Internal method that processes a single frame for DMS analysis.
        """
        is_sleeping = False
        is_distracted = False
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.detector(gray)
        if len(faces) == 0:
            cv2.putText(img, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        for face in faces:
            landmarks = self.predictor(gray, face)
            nose = (landmarks.part(30).x, landmarks.part(30).y)
            left_ear = (landmarks.part(1).x, landmarks.part(1).y)
            right_ear = (landmarks.part(15).x, landmarks.part(15).y)
            left_distance = np.linalg.norm(np.array(nose) - np.array(left_ear))
            right_distance = np.linalg.norm(np.array(nose) - np.array(right_ear))
            yaw_ratio = right_distance / (left_distance + right_distance)
            raw_yaw_angle = (yaw_ratio - 0.5) * 90
            yaw_angle = self.kalman_filter.predict()[0]
            yaw_angle = self.kalman_filter.correct(np.array([[raw_yaw_angle]], np.float32))[0][0]
            
            cv2.putText(img, f"Yaw: {yaw_angle:.1f}° (Raw: {raw_yaw_angle:.1f}°)", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            if abs(yaw_angle) < self.SIDE_FACE_ANGLE_THRESHOLD:
                self.side_face_start_time = None
                self.side_face_duration = 0
                
                x, y, w, h = face.left(), face.top(), face.width(), face.height()
                cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(img, "FRONT FACE", (int(img.shape[1]/2)-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                is_sleeping = self._process_eye_detection(img, landmarks)
                
            else:
                x, y, w, h = face.left(), face.top(), face.width(), face.height()
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 255), 2)
                face_direction = "Left" if yaw_angle > 0 else "Right"
                cv2.putText(img, f"SIDE FACE - Looking {face_direction}", 
                           (int(img.shape[1]/2)-150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                if self.side_face_start_time is None:
                    self.side_face_start_time = time.time()
                
                self.side_face_duration = time.time() - self.side_face_start_time
                countdown = max(0, self.SIDE_FACE_THRESHOLD - self.side_face_duration)
                cv2.putText(img, f"Distraction alert in: {countdown:.1f}s", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                if self.side_face_duration >= self.SIDE_FACE_THRESHOLD:
                    is_distracted = True
                    cv2.putText(img, "DRIVER DISTRACTED!", 
                               (int(img.shape[1]/2)-150, int(img.shape[0]/2)), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                    print(f"DMS: Driver distracted (side face) - {self.side_face_duration:.2f}s")
                    
                    # Reset sleep detection when distracted
                    self.closed_eye_start_time = None
                    self.closed_eye_duration = 0
                    self.open_eye_start_time = None
                    self.open_eye_duration = 0
        
        status_text = []
        if is_sleeping:
            status_text.append("SLEEPING")
        if is_distracted:
            status_text.append("DISTRACTED")
        
        if status_text:
            cv2.putText(img, "Driver Status: " + " & ".join(status_text), 
                       (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(img, "Driver Status: OK", 
                       (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return is_sleeping, is_distracted, img

    def _process_eye_detection(self, img, landmarks):
        """
        Service Name: _process_eye_detection
        Sync/Async: Synchronous
        Reentrancy: Reentrant
        Parameters (in): img - Camera frame, landmarks - Face landmarks
        Parameters (inout): None
        Parameters (out): None
        Return value: is_sleeping - Boolean indicating if driver is sleeping
        Description: processing eye regions to detect drowsiness.
        """
        is_sleeping = False
        
        try:
            # eye regions
            left_eye_pts = np.array([(landmarks.part(n).x, landmarks.part(n).y) for n in range(17, 22)] +  
                                   [(landmarks.part(n).x, landmarks.part(n).y) for n in range(36, 42)] +  
                                   [(landmarks.part(n).x, landmarks.part(n).y) for n in range(28, 31)])
            right_eye_pts = np.array([(landmarks.part(n).x, landmarks.part(n).y) for n in range(22, 27)] +  
                                    [(landmarks.part(n).x, landmarks.part(n).y) for n in range(42, 48)] +  
                                    [(landmarks.part(n).x, landmarks.part(n).y) for n in range(29, 31)])

            x_left, y_left, w_left, h_left = cv2.boundingRect(left_eye_pts)
            x_right, y_right, w_right, h_right = cv2.boundingRect(right_eye_pts)

            expand_ratio = 0.2
            h_expand = int(h_left * expand_ratio)
            y_left = max(y_left - h_expand, 0)
            h_left = min(h_left + h_expand, img.shape[0] - y_left)
            y_right = max(y_right - h_expand, 0)
            h_right = min(h_right + h_expand, img.shape[0] - y_right)
            
            left_eye = img[y_left:y_left+h_left, x_left:x_left+w_left]
            right_eye = img[y_right:y_right+h_right, x_right:x_right+w_right]
            
            if left_eye.size > 0 and right_eye.size > 0:
                cv2.imwrite("archive/person_eye_left.jpg", left_eye)
                cv2.imwrite("archive/person_eye_right.jpg", right_eye)
                
                left_eye_class = self.predict_image_fromPath("archive/person_eye_left.jpg")
                right_eye_class = self.predict_image_fromPath("archive/person_eye_right.jpg")
                
                left_color = (0, 255, 0) if left_eye_class == "Open-Eyes" else (0, 0, 255)
                right_color = (0, 255, 0) if right_eye_class == "Open-Eyes" else (0, 0, 255)
                
                cv2.rectangle(img, (x_left, y_left), (x_left + w_left, y_left + h_left), left_color, 2)
                cv2.rectangle(img, (x_right, y_right), (x_right + w_right, y_right + h_right), right_color, 2)
                cv2.putText(img, left_eye_class, (x_left, y_left - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, left_color, 1)
                cv2.putText(img, right_eye_class, (x_right, y_right - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, right_color, 1)

                if left_eye_class == "Close-Eyes" and right_eye_class == "Close-Eyes":
                    if self.closed_eye_start_time is None:
                        self.closed_eye_start_time = time.time()
                    self.closed_eye_duration = time.time() - self.closed_eye_start_time
                    countdown = max(0, self.CLOSED_EYE_THRESHOLD - self.closed_eye_duration)
                    cv2.putText(img, f"Sleep alert in: {countdown:.1f}s", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    if self.closed_eye_duration >= self.CLOSED_EYE_THRESHOLD:
                        is_sleeping = True
                        cv2.putText(img, "DRIVER SLEEPING!", 
                                   (int(img.shape[1]/2)-150, int(img.shape[0]/2)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                        print(f"DMS: Driver sleeping detected - {self.closed_eye_duration:.2f}s")
                        
                       
                        self.open_eye_start_time = None
                        self.open_eye_duration = 0
                        self.side_face_start_time = None
                        self.side_face_duration = 0
                else:
                    if self.open_eye_start_time is None:
                        self.open_eye_start_time = time.time()
                    self.open_eye_duration = time.time() - self.open_eye_start_time
                    if self.open_eye_duration >= self.OPEN_EYE_THRESHOLD:
                        self.closed_eye_start_time = None
                        self.closed_eye_duration = 0
                        
        except Exception as e:
            print(f"DMS: Error in eye detection: {e}")
        
        return is_sleeping

    def update(self):
        """
        Service Name: update
        Sync/Async: Asynchronous
        Reentrancy: Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: bool
        Description: Non-blocking update that displays the latest processed frame.
        """
        # Check for timeout to reset status
        current_time = time.time()
        with self._status_lock:
            if current_time - self.last_update_time > self.update_timeout:
                self.is_driver_sleeping = False
                self.is_driver_distracted = False
        try:
            processed_frame = self.result_queue.get(block=False)
            cv2.imshow("Driver Monitoring", processed_frame)
            cv2.waitKey(1)
            return True
        except queue.Empty:
            return True

    def is_sleeping(self):
        """
        Service Name: is_sleeping
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: bool
        Description: Thread-safe method to check if driver is sleeping.
        """
        with self._status_lock:
            return self.is_driver_sleeping

    def is_distracted(self):
        """
        Service Name: is_distracted
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: bool
        Description: Thread-safe method to check if driver is distracted.
        """
        with self._status_lock:
            return self.is_driver_distracted

    def _create_kalman_filter(self):
        """
        Service Name: _create_kalman_filter
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): None
        Parameters (inout): None
        Parameters (out): None
        Return value: cv2.KalmanFilter
        Description: Initializes and returns a Kalman filter for yaw angle estimation.
        """
        kalman = cv2.KalmanFilter(2, 1)
        kalman.measurementMatrix = np.array([[1, 0]], np.float32)
        kalman.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
        kalman.processNoiseCov = np.array([[0.1, 0], [0, 0.01]], np.float32)
        kalman.statePre = np.array([[0], [0]], np.float32)
        return kalman
    
    def _load_class_mapping(self):
        with open("Gclass_mapping.pickle", "rb") as f:
            return pickle.load(f)
    
    def _load_model(self):
        return pickle.load(open("SVM_model.sav", "rb"))
    
    def extract_hog_features(self, image, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2)):
        """
        Service Name: extract_hog_features
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): image - Input image, orientations - Number of orientations, pixels_per_cell - Size of each cell, cells_per_block - Number of cells per block
        Parameters (inout): None
        Parameters (out): None
        Return value: hog_features - Extracted HOG features
        Description: Extracts HOG features from the input image.
        """
        hog_features = hog(image, orientations=orientations,
                          pixels_per_cell=pixels_per_cell,
                          cells_per_block=cells_per_block,
                          block_norm='L2-Hys', visualize=False)
        return hog_features

    def preprocess_image_fromPath(self, image_path, target_size=(64, 64)): 
        """
        Service Name: preprocess_image_fromPath
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): image_path - Path to the input image, target_size - Size to resize the image
        Parameters (inout): None
        Parameters (out): None
        Return value: normalized - Preprocessed image ready for feature extraction
        Description: preprocessing the input image by converting to grayscale, applying Gaussian and median blur, resizing, and normalizing.
        """    
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError(f"Image not found at {image_path}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        filtered = cv2.medianBlur(blurred, 3)
        resized = cv2.resize(filtered, target_size)
        normalized = resized / 255.0
        return normalized
    
    def predict_image_fromPath(self, image_path, target_size=(64, 64)):
        """
        Service Name: predict_image_fromPath
        Sync/Async: Synchronous
        Reentrancy: Non-Reentrant
        Parameters (in): image_path - Path to the input image, target_size - Size to resize the image
        Parameters (inout): None
        Parameters (out): None
        Return value: class_name - Predicted class name for the input image
        Description: predicting the class of the input image using the pre-trained model and returns the class name.
        """
        if self.model is None:
            return "Open-Eyes"  
        preprocessed_image = self.preprocess_image_fromPath(image_path, target_size)
        feature_vector = self.extract_hog_features(preprocessed_image).reshape(1, -1)
        prediction = self.model.predict(feature_vector)[0]
        for class_name, label in self.class_mapping.items():
            if label == prediction:
                return class_name
        return "Open-Eyes"

    def cleanup(self):
        """
        Service Name      : cleanup
        Sync/Async        : Synchronous
        Reentrancy        : Non-Reentrant
        Parameters (in)   : None
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : None
        Description       : Releases the video capture and destroys all OpenCV windows.
        """
        self.cap.release()
        cv2.destroyAllWindows()

    def destroy(self):
        """
        Service Name      : destroy
        Sync/Async        : Synchronous
        Reentrancy        : Non-Reentrant
        Parameters (in)   : None
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : None
        Description       : Properly shuts down the DMS module and stops the background thread.
        """
        self.thread_active = False
        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)
        self.cleanup()
        print("DMS module destroyed")