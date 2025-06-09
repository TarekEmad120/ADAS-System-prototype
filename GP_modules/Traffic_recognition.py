import sys
import cv2
import numpy as np
import tensorflow as tf
import time
import os
from PIL import Image
import argparse

class TrafficSignDetector:
    '''
    * Service Name: TrafficSignDetector
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): model_path, confidence_threshold, min_detections, time_window, process_every_n_frames
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to detect and classify traffic signs in a video stream using a pre-trained CNN model based
    *                on the LeNet architecture. The detector uses a combination of color-based segmentation.
    '''
    def __init__(self, model_path, confidence_threshold=0.7, min_detections=3, time_window=0.8, process_every_n_frames=2):
        """
        Initialize traffic sign detector with temporal filtering
        Args:
            model_path: Path to trained classification model
            confidence_threshold: Minimum confidence to consider a detection valid
            min_detections: Minimum number of detections needed within time window to report a sign
            time_window: Time window in seconds for temporal filtering
            process_every_n_frames: Process only every Nth frame for speed
        """
        print(f"Loading model from {model_path}...")
        self.model = tf.keras.models.load_model(model_path)
        print("Model loaded successfully")
        
        self.detector_initialized = False
        self.confidence_threshold = confidence_threshold
        self.process_every_n_frames = process_every_n_frames
        self.frame_counter = 0
        
        # classes names
        self.classes = { 
            0:'Speed limit (20km/h)', 1:'Speed limit (30km/h)', 
            2:'Speed limit (50km/h)', 3:'Speed limit (60km/h)', 
            4:'Speed limit (70km/h)', 5:'Speed limit (80km/h)', 
            6:'Speed limit (90km/h)', 7:'Speed limit (100km/h)', 
            8:'Speed limit (120km/h)', 9:'No passing', 
            10:'No passing veh over 3.5 tons', 11:'Right-of-way at intersection', 
            12:'Priority road', 13:'Yield', 14:'Stop', 
            15:'No vehicles', 16:'Veh > 3.5 tons prohibited', 
            17:'No entry', 18:'General caution', 
            19:'Dangerous curve left', 20:'Dangerous curve right', 
            21:'Double curve', 22:'Bumpy road', 
            23:'Slippery road', 24:'Road narrows on the right', 
            25:'Road work', 26:'Traffic signals', 
            27:'Pedestrians', 28:'Children crossing', 
            29:'Bicycles crossing', 30:'Beware of ice/snow',
            31:'Wild animals crossing', 32:'End speed + passing limits', 
            33:'Turn right ahead', 34:'Turn left ahead', 
            35:'Ahead only', 36:'Go straight or right', 
            37:'Go straight or left', 38:'Keep right', 
            39:'Keep left', 40:'Roundabout mandatory', 
            41:'End of no passing', 42:'End no passing veh > 3.5 tons' 
        }
        
        self.detection_history = {}  
        self.min_detections = min_detections
        self.time_window = time_window
        self.stable_detections = {} 
        self.use_roi = True
        self.roi_x_start = 0
        self.roi_y_start = 0
        self.roi_width = 1.0  
        self.roi_height = 0.6 
        
    '''
    * Service Name: _initialize_detector
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): frame
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: Initialize the detector parameters based on the first frame
    *               This includes setting the region of interest (ROI) for detection.
    '''
    def _initialize_detector(self, frame):
        h, w = frame.shape[:2]
        self.roi_x_start = int(w * 0.1) 
        self.roi_width = int(w * 0.8)    
        self.roi_y_start = 0            
        self.roi_height = int(h * 0.6)  
        self.detector_initialized = True
        
    '''
    * Service Name: detect_potential_signs
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): frame
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: potential_signs
    * Description: Detect potential traffic signs in the given frame using color segmentation.
    *               The function uses HSV color space to create masks for red and blue signs.
    '''
    def detect_potential_signs(self, frame):
        if self.use_roi:
            x_end = self.roi_x_start + self.roi_width
            y_end = self.roi_y_start + self.roi_height
            roi = frame[self.roi_y_start:y_end, self.roi_x_start:x_end]
        else:
            roi = frame
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detect red signs
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Detect blue signs
        lower_blue = np.array([100, 80, 80])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)


        mask = cv2.bitwise_or(red_mask, blue_mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        potential_signs = []
        min_area = 400
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
    
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if 0.7 <= aspect_ratio <= 1.3: 
                if self.use_roi:
                    x += self.roi_x_start
                    y += self.roi_y_start

                potential_signs.append((x, y, w, h))
                
        return potential_signs
    '''
    * Service Name: classify_sign
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): frame, x, y, w, h
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: class_id, confidence
    * Description: Classify the detected sign using the pre-trained model.
    ''' 
    def classify_sign(self, frame, x, y, w, h):

        sign_roi = frame[y:y+h, x:x+w]        
        try:
            rgb_roi = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb_roi)
            resized_img = pil_img.resize((32, 32))
            img_array = np.array(resized_img) / 255.0
            img_array = np.expand_dims(img_array, axis=0)
            predictions = self.model.predict(img_array, verbose=0)
            class_id = np.argmax(predictions[0])
            confidence = predictions[0][class_id]
            return class_id, confidence
        except Exception as e:
            print(f"Error classifying sign: {e}")
            return None, 0.0
    '''
    * Service Name: add_detection
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): class_id, confidence, box, timestamp
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: Add a detection to the history for temporal filtering.
    *              The function keeps only the detections within the time window.
    *              This helps in reducing false positives and stabilizing the detection.
    '''      
    def add_detection(self, class_id, confidence, box, timestamp):
        if class_id not in self.detection_history:
            self.detection_history[class_id] = []
        self.detection_history[class_id].append((timestamp, confidence, box))
        self.detection_history[class_id] = [
            d for d in self.detection_history[class_id]
            if timestamp - d[0] <= self.time_window
        ]

    '''
    * Service Name: get_stable_detections
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): current_time
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: stable_detections
    * Description: Get signs that have been consistently detected within the time window.
    *              The function calculates the average confidence and uses the most recent box position.
    '''    
    def get_stable_detections(self, current_time):
        self.stable_detections = {}

        for class_id, detections in self.detection_history.items():
            recent = [d for d in detections if current_time - d[0] <= self.time_window]
            if len(recent) >= self.min_detections:
                avg_conf = sum(conf for _, conf, _ in recent) / len(recent)
                self.stable_detections[class_id] = (avg_conf, recent[-1][2])
                
        return self.stable_detections

    '''
    * Service Name: process_frame
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): frame
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: output_frame
    * Description: Process a single frame to detect and classify traffic signs.
    *              The function uses the initialized detector to find potential signs,
    '''
    def process_frame(self, frame):
        if not self.detector_initialized:
            self._initialize_detector(frame)
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            output_frame = frame.copy()
            self._draw_stable_detections(output_frame)
            return output_frame
        current_time = time.time()
        output_frame = frame.copy()
        potential_signs = self.detect_potential_signs(frame)

        for x, y, w, h in potential_signs:
            class_id, confidence = self.classify_sign(frame, x, y, w, h)
            if confidence >= self.confidence_threshold:
                self.add_detection(class_id, confidence, (x, y, w, h), current_time)
        
     
        self.get_stable_detections(current_time)
        self._draw_stable_detections(output_frame)
        return output_frame
    
    '''
    * Service Name: _draw_stable_detections
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): frame
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: Draw the stable detections on the frame.
    '''
    def _draw_stable_detections(self, frame):
        for class_id, (confidence, (x, y, w, h)) in self.stable_detections.items():
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = self.classes.get(class_id, "Unknown")
            label_text = f"{label}: {confidence:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 2
            size, _ = cv2.getTextSize(label_text, font, font_scale, thickness)
            cv2.rectangle(frame, (x, y-size[1]-5), (x+size[0], y), (0, 0, 0), -1)
            cv2.putText(frame, label_text, (x, y-5), font, font_scale, (255, 255, 255), thickness)
        if self.use_roi:
            x_end = self.roi_x_start + self.roi_width
            y_end = self.roi_y_start + self.roi_height
            cv2.rectangle(frame, (self.roi_x_start, self.roi_y_start), (x_end, y_end), (255, 0, 0), 2)

def main():
    if 'ipykernel' in sys.modules:
        video_source = r"D:\gp project videos\traffic signs test.mp4"
        model_path = r"D:\GpModules\Traffic_Recognition\LeNETModel.h5"
        confidence_threshold = 0.85
        skip_frames = 1
        min_detections = 4
    else:
        parser = argparse.ArgumentParser(description='Traffic Sign Detection and Classification')
        parser.add_argument('--video', type=str, default=0, help='Path to video file or camera index')
        parser.add_argument('--model', type=str, default='traffic_sign_model.h5', help='Path to trained model')
        parser.add_argument('--confidence', type=float, default=0.7, help='Confidence threshold')
        parser.add_argument('--skip-frames', type=int, default=2, help='Process every Nth frame')
        parser.add_argument('--min-detections', type=int, default=3, help='Minimum detections to consider stable')
        args = parser.parse_args()
        video_source = args.video
        model_path = args.model
        confidence_threshold = args.confidence
        skip_frames = args.skip_frames
        min_detections = args.min_detections

    detector = TrafficSignDetector(
        model_path=model_path,
        confidence_threshold=confidence_threshold,
        min_detections=min_detections,
        process_every_n_frames=skip_frames
    )

    try:
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            raise Exception(f"Cannot open video source {video_source}")
        
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_delay = int(1000 / fps)
        
        fps_values = [] 
        while True:
            ret, frame = cap.read()
            if not ret:
                print("End of video or error reading frame")
                break
            start_time = time.time()
            processed_frame = detector.process_frame(frame)
            process_time = time.time() - start_time
            if process_time > 0: 
                current_fps = 1.0 / process_time
                fps_values.append(current_fps)
                if len(fps_values) > 10:
                    fps_values.pop(0)
                avg_fps = sum(fps_values) / len(fps_values)
                cv2.putText(processed_frame, f"FPS: {avg_fps:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Traffic Sign Detection", processed_frame)
            if cv2.waitKey(frame_delay) & 0xFF == 27:
                break
                
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()