# Real-Time Autonomous Vehicle 🚗

## 📌 Overview
This project implements a real-time autonomous vehicle system that combines sensor fusion and machine learning-based lane detection for intelligent navigation.

## 🛠️ Technologies Used
- Raspberry Pi
- QNX RTOS (SCHED_FIFO scheduling)
- Ultrasonic Sensors
- IMU (Inertial Measurement Unit)
- Python
- PyTorch
- YOLO (Object Detection / Lane Detection)

## ⚙️ Features
- Real-time obstacle detection using ultrasonic sensors
- Sensor fusion combining IMU, encoders, and distance sensors
- Priority-based multi-threading using QNX RTOS for deterministic execution
- ML-based lane detection using YOLO
- Emergency override system for safety
- Dynamic navigation (forward / left / right / stop decisions)

## 🔄 How It Works
Sensor data (ultrasonic + IMU + encoders) is fused → processed in real-time using RTOS scheduling → decision logic determines navigation → YOLO model enhances lane detection → vehicle adjusts path accordingly.

## 🧠 Machine Learning Component
- YOLO-based lane detection model
- Trained on custom dataset (144 images)
- Achieved ~63% mIoU for lane segmentation
- Enhances navigation accuracy alongside sensor-based obstacle avoidance

## 🛡️ Safety Mechanisms
- Anomaly detection using:
  - Threshold checks
  - Rate-of-change validation
  - Sensor consistency verification
- Emergency override system to stop vehicle instantly

## 📸 Project Demo
(Add images/videos here)
- Vehicle setup
- Sensor layout
- Lane detection output

## ⚡ Future Improvements
- Improve dataset size for better ML accuracy
- Add LiDAR for advanced perception
- Implement SLAM for mapping
- Real-time dashboard monitoring

## 🧠 Learning Outcomes
- Real-time embedded system design
- RTOS-based task scheduling
- Sensor fusion techniques
- Computer vision with YOLO
- Autonomous navigation logic

## 👨‍💻 Author
Kishore Kanna P
