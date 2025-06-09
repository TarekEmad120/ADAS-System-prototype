## Features

* **YOLOv8 Object Detection**: Detects general categories of German traffic signs and traffic lights in real time.
* **SVM Classification**: Classifies detected signs into 40+ specific categories using HOG features.
* **Data Structuring**: Automatically processes and structures YOLO datasets.
* **Image Augmentation**: Uses Gaussian blur and rotation to improve SVM robustness.

---

## Requirements

Install required Python packages:

```bash
pip install -r requirements.txt
```

**`requirements.txt`**:

```text
opencv-python
ultralytics
numpy
scikit-learn
scikit-image
joblib
```

---
## Datasets

#### Original Datasets
https://www.kaggle.com/datasets/meowmeowmeowmeowmeow/gtsrb-german-traffic-sign

https://www.kaggle.com/datasets/valentynsichkar/traffic-signs-dataset-in-yolo-format

https://www.kaggle.com/datasets/sachsene/carla-traffic-lights-images

https://www.kaggle.com/datasets/sachsene/carla-traffic-signs-images/data

https://universe.roboflow.com/jung-4vwhz/erp_traffic_light/dataset/1

https://universe.roboflow.com/sri-sudhi/traffic_light-ebhq9

#### Reconstructed Datasets
##### YOLO:
    
TSR: https://www.kaggle.com/datasets/osamaelhattab/traffic-signs-structured-dataset-for-yolo
    
TSR & TLR: https://www.kaggle.com/datasets/osamaelhattab/traffic-signs-and-traffic-lights-classes-for-yolo

##### SVM:

TSR & TLR: https://www.kaggle.com/datasets/osamaelhattab/german-traffic-signs-with-traffic-lights-dataset

---
## Trained Models
https://drive.google.com/drive/folders/1-uWfq53Bh7iAdO5Y27x7lQrFd0EmRBzU?usp=drive_link

---
## Workflow Overview

### 1. Prepare YOLO Dataset

you can find the link for the prepared datasets above

---

### 2. Train YOLOv8

```python
model.train(data="structured_yolo_dataset_root/data.yaml", epochs=5, imgsz=960, batch=16)
```

Uses `yolov8n.yaml` config and `yolov8n.pt` pretrained weights.

---

### 3. Train SVM with HOG Features

```python
X, y = preprocess_and_extract_features(images, labels)
clf.fit(X_train, y_train)
```

Images are resized, converted to grayscale, and HOG features are extracted.

---

### 4. Real-Time Detection

* YOLOv8 detects objects in the video.
* Cropped signs are resized and passed to the SVM.
* SVM classifies the sign and overlays the result on the video frame.

---

## Output

Each frame is displayed with bounding boxes and labels, for example:

```
[✓] Stop (0.91)
[✓] Speed Limit 30 (0.87)
```

Confidence scores are shown next to predictions.

---

## Supported Classes

The SVM model classifies over 40 traffic signs, including:

Full list of supported classes:

```python
CLASS_NAMES = [
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
```

---

## How to Run

Ensure the video file is present at:

```bash
./CarlaVideos/traffic signs test.mp4
```

Run the script:

```bash
python your_script.py
```

Press `q` to exit the OpenCV display window.

---

## References

* [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
* [scikit-learn SVM Documentation](https://scikit-learn.org/stable/modules/svm.html)
* [HOG Feature Extraction – scikit-image](https://scikit-image.org/docs/dev/api/skimage.feature.html#skimage.feature.hog)
* [OpenCV](https://opencv.org/)
* [Carla Simulator](https://carla.org/)