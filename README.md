# Vision-Hand Gesture Control System 🎯🖐️

A smart gesture-controlled interface that combines **computer vision** (MediaPipe + OpenCV) and **Arduino sensors** to control mouse movement, clicks, scrolling, and volume — all **touch-free**.



## 🔧 Key Features

- 🖱️ **Hand Tracking with MediaPipe** for cursor movement
- 👆 **Finger Gestures** to perform:
  - Left click (Thumb + Index)
  - Right click (Thumb + Middle)
  - Drag (Thumb + Ring)
  - Double click
- 🧭 **HC-SR04 Ultrasonic Sensors**:
  - Sensor 1: Control **volume**
  - Sensor 2: Control **scrolling**
- 🔄 **IR Sensor toggle**: Lock/Unlock cursor or enable/disable controls
- 🔌 **Auto-detect Arduino COM port**
- 📋 On-screen instructions and real-time feedback using OpenCV



## 💡 Technologies used in this proejct

| Technology       | Purpose                            |
|------------------|------------------------------------|
| Python + OpenCV  | Webcam processing, GUI overlay     |
| MediaPipe        | Hand tracking & landmark detection |
| PyAutoGUI        | Simulate mouse & scroll actions    |
| PyCAW            | Volume control on Windows          |
| Arduino + HC-SR04| Sensor input for distance reading  |
| IR Sensor        | Toggle lock/control states         |

---

## 🧠 How It Works

- The system uses **MediaPipe Hands** to detect hand landmarks in real-time.
- Based on specific finger gestures (like pinching), it simulates clicks or drag.
- Meanwhile, **ultrasonic sensors** connected to Arduino detect hand distance and send serial data:
  - Closer hand = higher volume or scroll up
  - Farther hand = lower volume or scroll down
- An **IR sensor** lets you toggle between locked/unlocked cursor states.

---

## 📦 Requirements

### Python Packages:
```
pip install opencv-python mediapipe numpy pyautogui pycaw pyserial comtypes
```
Arduino Hardware:
Arduino UNO/Nano

HC-SR04 x2

IR Sensor x1

USB cable

Arduino Sketch:
Upload the code in MPCA_mini_project.ino to your Arduino board.

▶️ Run the Program
Connect your Arduino with the required sensors.

Upload the Arduino .ino code.

Run the Python script:
```
python MPCA_mini_project.py
```
Use hand gestures and sensors to control your PC.

#📌 Controls Summary
Action	Trigger
Move Cursor	Thumb movement
Left Click	Thumb + Index finger pinch
Right Click	Thumb + Middle finger pinch
Drag	Thumb + Ring finger pinch & hold
Double Click	Quick double Thumb + Index
Scroll	Hand near Sensor 2 (HC-SR04)
Volume	Hand near Sensor 1 (HC-SR04)
Lock/Unlock Cursor	IR Sensor trigger

👨‍💻 Authors
Nandan D and team

📄 License
This project is for academic and experimental use only.
