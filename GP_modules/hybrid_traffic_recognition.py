import cv2
import numpy as np
from ultralytics import YOLO
import joblib
from skimage.feature import hog
import time
import threading
import queue

class HybridTrafficRecognizer:
    def __init__(self, yolo_model_path, svm_model_path, confidence_threshold=0.5):
        """
        Service Name      : HybridTrafficRecognizer
        Sync/Async        : Asynchronous
        Reentrancy        : Reentrant
        Parameters (in)   : yolo_model_path (str) - Path to the YOLO model file
                            svm_model_path (str) - Path to the SVM model file
                            confidence_threshold (float) - Minimum confidence for detections
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : None
        Description       : Initializes the HybridTrafficRecognizer with YOLO and SVM models.
                            Sets up threading for asynchronous processing.
        """
        self.yolo_model = YOLO(yolo_model_path)
        self.svm_model = joblib.load(svm_model_path)
        self.confidence_threshold = confidence_threshold
        self.CLASS_NAMES = [
            'Green Light', 'Red Light', 'Speed Limit 10', 'Speed Limit 100', 'Speed Limit 110',
            'Speed Limit 120', 'Speed Limit 20', 'Speed Limit 30', 'Speed Limit 40', 'Speed Limit 50',
            'Speed Limit 60', 'Speed Limit 70', 'Speed Limit 80', 'Speed Limit 90', 'Stop',
            'No vehicles', 'Veh > 3.5 tons prohibited', 'No entry', 'General caution',
            'Dangerous curve left', 'Dangerous curve right', 'Double curve', 'Bumpy road',
            'Slippery road', 'Road narrows on the right', 'Road work', 'Traffic signals',
            'Pedestrians', 'Children crossing', 'Bicycles crossing', 'Beware of ice/snow',
            'Wild animals crossing', 'End speed + passing limits', 'Turn right ahead', 'Turn left ahead',
            'Ahead only', 'Go straight or right', 'Go straight or left', 'Keep right',
            'Keep left', 'Roundabout mandatory', 'End of no passing', 'End no passing veh > 3.5 tons'
        ]
        
        # Detection storage with timestamps
        self.recent_detections = {}
        self.detection_history = {}
        self.stable_detections = {}
        
        # Threading components
        self.frame_queue = queue.Queue(maxsize=2)
        self.result_queue = queue.Queue(maxsize=2)
        self.thread_active = True
        self.processing_thread = threading.Thread(target=self._processing_thread_function, daemon=True)
        self.processing_thread.start()
        
        # Frame processing control
        self.frame_counter = 0
        self.process_every_n_frames = 3
        
        # Detection caching with expiration
        self.latest_detections = []
        self.last_detection_time = 0
        self.detection_timeout = 2.0  # Clear detections after 2 seconds of no new detections
        
        print("Hybrid Traffic Recognition initialized in threaded mode")
        
    def _processing_thread_function(self):
        """Background thread that processes frames from the queue"""
        while self.thread_active:
            try:
                try:
                    frame = self.frame_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                detections = self._process_frame_internal(frame)
                timestamp = time.time()
                detection_result = (detections, timestamp)
                
                try:
                    self.result_queue.put(detection_result, block=False)
                except queue.Full:
                    try:
                        self.result_queue.get_nowait()
                        self.result_queue.put(detection_result, block=False)
                    except queue.Empty:
                        pass
                
                self.frame_queue.task_done()             
            except Exception as e:
                print(f"Error in traffic recognition thread: {e}")
                time.sleep(0.1)
        
        print("Traffic recognition thread terminated")
        
    def process_frame(self, frame):
        """
        Service Name      : process_frame
        Sync/Async        : Asynchronous
        Reentrancy        : Reentrant
        Parameters (in)   : frame - Input camera frame
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : processed_frame - Frame with detections drawn
                            detections - List of detection tuples
        Description       : Queues a frame for processing and returns frame with current detections
        """
        current_time = time.time()
        self._cleanup_expired_detections(current_time)
        clean_frame = frame.copy()
        self.frame_counter += 1
        should_process = (self.frame_counter % self.process_every_n_frames == 0)
        
        if should_process:
            try:
                self.frame_queue.put(clean_frame, block=False)
            except queue.Full:
                pass
        try:
            detections, detection_time = self.result_queue.get(block=False)
            if current_time - detection_time < self.detection_timeout:
                self.latest_detections = detections
                self.last_detection_time = detection_time
        except queue.Empty:
            pass
    
        if current_time - self.last_detection_time > self.detection_timeout:
            self.latest_detections = []
        if self.latest_detections:
            drawn_frame = self._draw_detections_on_frame(clean_frame, self.latest_detections)
            return drawn_frame, self.latest_detections
        else:
            return clean_frame, []
    
    def _cleanup_expired_detections(self, current_time):
        """
        Service Name      : _cleanup_expired_detections
        Sync/Async        : Synchronous
        Reentrancy        : Reentrant
        Parameters (in)   : current_time (float) - Current timestamp
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : None
        Description       : Cleans up expired detections from history and recent detections.
        """

        for class_id in list(self.detection_history.keys()):
            self.detection_history[class_id] = [
                d for d in self.detection_history[class_id] 
                if current_time - d[2] < 2.0
            ]
            if not self.detection_history[class_id]:
                del self.detection_history[class_id]
        for class_id in list(self.stable_detections.keys()):
            if class_id not in self.detection_history:
                del self.stable_detections[class_id]
        for class_id in list(self.recent_detections.keys()):
            if current_time - self.recent_detections[class_id][2] > 2.0:
                del self.recent_detections[class_id]
    
    def _draw_detections_on_frame(self, frame, detections):
        """Draw detections on a fresh copy of the frame"""
        drawn_frame = frame.copy()
        
        for prediction, confidence, (x1, y1, x2, y2) in detections:
            if prediction < len(self.CLASS_NAMES):
                class_name = self.CLASS_NAMES[prediction]
                cv2.rectangle(drawn_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                text = f"{class_name} ({confidence:.2f})"
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)

                cv2.rectangle(drawn_frame, (x1, y1 - text_size[1] - 10), 
                             (x1 + text_size[0], y1), (0, 0, 0), -1)

                cv2.putText(drawn_frame, text, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return drawn_frame
            
    def _process_frame_internal(self, frame):
        """Internal method that processes a frame for detection in the background thread"""
        detections = []
        
        try:
            results = self.yolo_model.predict(frame, conf=self.confidence_threshold)
            
            if results and len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if (x2 - x1 < 10) or (y2 - y1 < 10):
                        continue
                    cropped = frame[y1:y2, x1:x2]
                    if cropped.size == 0:
                        continue
                    resized = cv2.resize(cropped, (64, 64))
                    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                    hog_feat = hog(gray, pixels_per_cell=(8, 8), cells_per_block=(2, 2), feature_vector=True)
                    features = hog_feat.reshape(1, -1)
                    prediction = self.svm_model.predict(features)[0]
                    confidence = max(self.svm_model.predict_proba(features)[0])
                    if confidence < 0.8:
                        continue
                    self._update_detection_history(prediction, confidence, (x1, y1, x2-x1, y2-y1))
                    detections.append((prediction, confidence, (x1, y1, x2, y2)))
                    
        except Exception as e:
            print(f"Error in traffic sign detection processing: {e}")
            
        return detections
    
    def _update_detection_history(self, class_id, confidence, bbox):
        """
        service Name      : _update_detection_history
        Sync/Async        : Synchronous
        Reentrancy        : Reentrant
        Parameters (in)   : class_id (int) - Class ID of the detected object
                            confidence (float) - Confidence score of the detection
                            bbox (tuple) - Bounding box coordinates (x1, y1, width, height) 
        Parameters (inout): None
        Parameters (out)  : None
        Return value      : None
        Description       : Updates the detection history and stable detections for a given class ID.
        """
        current_time = time.time()
        
        self.recent_detections[class_id] = (confidence, bbox, current_time)
        if class_id not in self.detection_history:
            self.detection_history[class_id] = []
            
        self.detection_history[class_id].append((confidence, bbox, current_time))
        self.detection_history[class_id] = [
            d for d in self.detection_history[class_id] 
            if current_time - d[2] < 2.0
        ]
        if len(self.detection_history[class_id]) >= 3:
            avg_conf = sum(d[0] for d in self.detection_history[class_id]) / len(self.detection_history[class_id])
            self.stable_detections[class_id] = (avg_conf, bbox)
        elif class_id in self.stable_detections and len(self.detection_history[class_id]) == 0:
            del self.stable_detections[class_id]
                
    def destroy(self):
        """Clean up resources and stop the background thread"""
        self.thread_active = False
        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=1.0)
        print("Traffic recognition thread cleaned up")