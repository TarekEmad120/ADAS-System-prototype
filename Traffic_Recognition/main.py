import cv2
import numpy as np
import tensorflow as tf
import time
import os
from PIL import Image
import argparse

class TrafficSignDetector:
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
        # Load trained model
        print(f"Loading model from {model_path}...")
        self.model = tf.keras.models.load_model(model_path)
        print("Model loaded successfully")
        
        # Sign detector: Using color and shape detection for speed
        self.detector_initialized = False
        
        # Detection parameters
        self.confidence_threshold = confidence_threshold
        self.process_every_n_frames = process_every_n_frames
        self.frame_counter = 0
        
        # Define class names
        self.classes = { 
            0:'Speed limit (20km/h)', 1:'Speed limit (30km/h)', 
            2:'Speed limit (50km/h)', 3:'Speed limit (60km/h)', 
            4:'Speed limit (70km/h)', 5:'Speed limit (80km/h)', 
            6:'End of speed limit (80km/h)', 7:'Speed limit (100km/h)', 
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
        
        # Temporal filtering for detection stability
        self.detection_history = {}  # {class_id: [(timestamp, confidence, box), ...]}
        self.min_detections = min_detections
        self.time_window = time_window
        self.stable_detections = {}  # Currently visible stable detections
        
        # Region of interest parameters - focus on where signs typically appear
        self.use_roi = True
        self.roi_x_start = 0
        self.roi_y_start = 0
        self.roi_width = 1.0  # Will be calculated as fraction of frame width
        self.roi_height = 0.6  # Will be calculated as fraction of frame height
        
    def _initialize_detector(self, frame):
        """Initialize detector parameters based on first frame"""
        h, w = frame.shape[:2]
        
        # Calculate ROI dimensions
        self.roi_x_start = int(w * 0.1)  # Start 10% from left
        self.roi_width = int(w * 0.8)    # Use middle 80% of width
        self.roi_y_start = 0             # Start from top
        self.roi_height = int(h * 0.6)   # Use top 60% of height
        
        self.detector_initialized = True
        
    def detect_potential_signs(self, frame):
        """Detect potential traffic sign regions using color and shape detection"""
        # Work on ROI if enabled
        if self.use_roi:
            x_end = self.roi_x_start + self.roi_width
            y_end = self.roi_y_start + self.roi_height
            roi = frame[self.roi_y_start:y_end, self.roi_x_start:x_end]
        else:
            roi = frame
            
        # Convert to HSV for better color detection
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
        
        # Combine masks
        mask = cv2.bitwise_or(red_mask, blue_mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size and shape
        potential_signs = []
        min_area = 400  # Minimum area to consider
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < min_area:
                continue
                
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Filter by aspect ratio (signs are roughly square)
            aspect_ratio = float(w) / h
            if 0.7 <= aspect_ratio <= 1.3:  # Close to square
                # Adjust coordinates to full frame if using ROI
                if self.use_roi:
                    x += self.roi_x_start
                    y += self.roi_y_start
                    
                potential_signs.append((x, y, w, h))
                
        return potential_signs
        
    def classify_sign(self, frame, x, y, w, h):
        """Classify a detected region using the CNN model"""
        # Extract region
        sign_roi = frame[y:y+h, x:x+w]
        
        # Prepare for model
        try:
            # Convert to RGB (our model expects RGB)
            rgb_roi = cv2.cvtColor(sign_roi, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL and resize
            pil_img = Image.fromarray(rgb_roi)
            resized_img = pil_img.resize((30, 30))  # Model input size
            
            # Normalize and expand dimensions for batch
            img_array = np.array(resized_img) / 255.0
            img_array = np.expand_dims(img_array, axis=0)
            
            # Make prediction
            predictions = self.model.predict(img_array, verbose=0)
            class_id = np.argmax(predictions[0])
            confidence = predictions[0][class_id]
            
            return class_id, confidence
            
        except Exception as e:
            print(f"Error classifying sign: {e}")
            return None, 0.0
            
    def add_detection(self, class_id, confidence, box, timestamp):
        """Add a new sign detection to history for temporal filtering"""
        if class_id not in self.detection_history:
            self.detection_history[class_id] = []
            
        # Add current detection
        self.detection_history[class_id].append((timestamp, confidence, box))
        
        # Remove old detections
        self.detection_history[class_id] = [
            d for d in self.detection_history[class_id]
            if timestamp - d[0] <= self.time_window
        ]
        
    def get_stable_detections(self, current_time):
        """Get signs that have been consistently detected"""
        self.stable_detections = {}
        
        for class_id, detections in self.detection_history.items():
            # Only consider recent detections
            recent = [d for d in detections if current_time - d[0] <= self.time_window]
            
            if len(recent) >= self.min_detections:
                # Calculate average confidence and position
                avg_conf = sum(conf for _, conf, _ in recent) / len(recent)
                
                # Use the most recent box position
                self.stable_detections[class_id] = (avg_conf, recent[-1][2])
                
        return self.stable_detections
        
    def process_frame(self, frame):
        """Process a single frame to detect traffic signs"""
        # Initialize detector parameters if not done yet
        if not self.detector_initialized:
            self._initialize_detector(frame)
            
        # Skip frames for performance
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            # Draw existing stable detections on skipped frames
            output_frame = frame.copy()
            self._draw_stable_detections(output_frame)
            return output_frame
            
        current_time = time.time()
        output_frame = frame.copy()
        
        # 1. Find potential sign regions
        potential_signs = self.detect_potential_signs(frame)
        
        # 2. Classify each potential sign
        for x, y, w, h in potential_signs:
            # Skip tiny regions (likely noise)
            if w < 20 or h < 20:
                continue
                
            class_id, confidence = self.classify_sign(frame, x, y, w, h)
            
            # Only consider high confidence detections
            if class_id is not None and confidence >= self.confidence_threshold:
                self.add_detection(class_id, confidence, (x, y, w, h), current_time)
        
        # 3. Update and draw stable detections
        self.get_stable_detections(current_time)
        self._draw_stable_detections(output_frame)
        
        return output_frame

    
    def _draw_stable_detections(self, frame):
        """Draw bounding boxes and labels for stable detections"""
        for class_id, (confidence, (x, y, w, h)) in self.stable_detections.items():
            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Get class name
            label = self.classes.get(class_id, "Unknown")
            
            # Draw label background
            label_text = f"{label}: {confidence:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 2
            size, _ = cv2.getTextSize(label_text, font, font_scale, thickness)
            
            cv2.rectangle(frame, (x, y-size[1]-5), (x+size[0], y), (0, 0, 0), -1)
            cv2.putText(frame, label_text, (x, y-5), font, font_scale, (255, 255, 255), thickness)
            
        # Optional: Draw ROI if enabled
        if self.use_roi:
            x_end = self.roi_x_start + self.roi_width
            y_end = self.roi_y_start + self.roi_height
            cv2.rectangle(frame, (self.roi_x_start, self.roi_y_start), (x_end, y_end), (255, 0, 0), 2)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Traffic Sign Detection and Classification')
    parser.add_argument('--video', type=str, default=0, help='Path to video file or camera index')
    parser.add_argument('--model', type=str, default='traffic_sign_model.h5', help='Path to trained model')
    parser.add_argument('--confidence', type=float, default=0.7, help='Confidence threshold')
    parser.add_argument('--skip-frames', type=int, default=2, help='Process every Nth frame')
    parser.add_argument('--min-detections', type=int, default=3, help='Minimum detections to consider stable')
    args = parser.parse_args()
    
    # Initialize detector
    model_path = r"D:\GpModules\Traffic_Recognition\model.h5"
    detector = TrafficSignDetector(
        model_path=model_path,
        confidence_threshold=args.confidence,
        min_detections=args.min_detections,
        process_every_n_frames=args.skip_frames
    )
    
    # Open video source
    try:
        video_source = r"D:\gp project videos\traffic signs test.mp4"
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            raise Exception(f"Cannot open video source {video_source}")
        
        # For FPS calculation
        prev_time = time.time()
        fps = 0
        fps_alpha = 0.1  # Smoothing factor
            
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Process frame
            start_time = time.time()
            processed_frame = detector.process_frame(frame)
            
            # Calculate FPS with smoothing and safeguard against division by zero
            process_time = time.time() - start_time
            if process_time > 0:  # Avoid division by zero
                current_fps = 1 / process_time
                fps = fps_alpha * current_fps + (1 - fps_alpha) * fps if fps > 0 else current_fps
            
            # Display FPS
            cv2.putText(processed_frame, f"FPS: {fps:.2f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display result
            cv2.imshow("Traffic Sign Detection", processed_frame)
            
            # Break on ESC key
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()