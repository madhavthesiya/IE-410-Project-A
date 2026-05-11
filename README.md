# IE410 – Project Part A: Object Manipulation using a Robotic Arm

> **Course:** IE410 Introduction to Robotics · Winter 2026  
> **Platform:** Arduino Braccio Robot Arm + Python + OpenCV

---

## Overview

This project implements three robotic manipulation tasks using the **Arduino Braccio** 6-DOF robot arm with a two-finger gripper. Tasks progress from basic open-loop control to camera-guided perception and dual-arm coordination.

| Task | Description | Method |
|------|-------------|--------|
| **Task 1** | Pre-Programmed Pick and Place | Hard-coded servo waypoints |
| **Task 2** | Camera-Assisted Pick and Place | ArUco markers + DroidCam + Python IK |
| **Task 3** | Object Handover Between Two Arms | Timed dual-arm coordination |

---

## Demo

🎥 **Video:** https://youtu.be/uWvt22I38Gw

---

## Repository Structure

```
├── Task1/
│   └── task1_preprogrammed.ino       # Direct servo angle pick-and-place
│
├── Task2/
│   ├── arduino_python_script3.ino    # Arduino: serial parser + IK + motion
│   ├── braccio_control_python.py     # Python: serial comm + camera compensation
│   ├── Aruco_detection_V2.py         # Python: DroidCam stream + ArUco detection
│   └── ArucoDetection_definitions.py # Python: marker utils + perspective warp
│
└── Task3/
    ├── task3_robot1.ino              # Robot 1: transport object to handover point
    └── task3_robot2.ino              # Robot 2: approach, receive, and retract
```

---

## Task 1 — Pre-Programmed Pick and Place

The arm picks an object from **Point A** and places it at **Point B** using a pre-defined sequence of six servo waypoints commanded via `Braccio.ServoMovement()`. No sensor input is required.

**Hardware:** Braccio arm + Arduino Uno  
**File:** `Task1/task1_preprogrammed.ino`

---

## Task 2 — Camera-Assisted Pick and Place

The object can be placed **anywhere** in the workspace. A smartphone camera (DroidCam) detects the object via ArUco markers and feeds its position to the arm in real time.

### How it works
1. Four **DICT_4X4_50** ArUco markers (IDs 1–4) are placed at workspace corners
2. One **DICT_6X6_50** ArUco marker is attached to the object
3. Python detects all markers, applies a **perspective warp** for a bird's-eye view, and maps pixel coordinates to robot-frame millimetres
4. A **foreshortening correction** compensates for the camera's elevated angle
5. Corrected Cartesian coordinates are sent over serial to the Arduino
6. The Arduino runs **geometric inverse kinematics** and executes a 9-step pick-and-place sequence

### Setup

**Python dependencies:**
```bash
pip install opencv-contrib-python pyserial numpy
```

**DroidCam setup:**
1. Install the [DroidCam app](https://www.dev47apps.com/) on your phone
2. Connect phone and PC to the **same Wi-Fi network**
3. Open `Aruco_detection_V2.py` and set:
```python
DROIDCAM_IP   = "YOUR_PHONE_IP"   # shown in DroidCam app
DROIDCAM_PORT = "4747"            # default port
SERIAL_PORT   = "COM12"           # your Arduino port (e.g. /dev/ttyUSB0 on Linux)
```

**Running:**
```bash
python Aruco_detection_V2.py
```
- Press **`p`** in the camera window to trigger a pick-and-place
- Press **`q`** to quit

**Arduino:** Flash `arduino_python_script3.ino` onto the Braccio Arduino at **115200 baud**

---

## Task 3 — Object Handover Between Two Robot Arms

Two Braccio arms transfer a soft object (crumpled paper ball) without dropping it. Coordination is achieved through calibrated time delays — no inter-robot communication hardware required.

| | Robot 1 | Robot 2 |
|---|---|---|
| **Role** | Picks and transports object to handover point | Waits, approaches, receives object |
| **Initial delay** | None | 12 seconds (waits for Robot 1 to stabilise) |
| **Overlap window** | Holds 5 s after Robot 2 grasps, then releases | Closes gripper during Robot 1's hold |
| **File** | `task3_robot1.ino` | `task3_robot2.ino` |

Flash each sketch to its respective Arduino Uno and power both arms simultaneously.

---

## Hardware Requirements

- Arduino Braccio Robot Arm × 2 (Task 3 requires two)
- Arduino Uno × 2
- Smartphone with [DroidCam](https://www.dev47apps.com/) installed (Task 2)
- PC with Python 3.8+ and Arduino IDE

---

## License

This project was developed for academic purposes as part of IE410 at Dhirubhai Ambani Univercity.
