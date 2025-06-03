# Smart Fire Suppression System with YOLOv5 and Pan-Tilt Control

---

## Project Overview

This project implements an automated fire suppression system utilizing a **YOLOv5** deep learning model for real-time fire detection, integrated with a **pan-tilt servo mechanism** for aiming and a **water pump** for suppression. The system uses a webcam feed to detect fires, calculates the fire's position relative to the camera's center, and then commands two servos (pan and tilt) to orient the camera and connected nozzle towards the fire. Once the fire is sufficiently centered, a water pump is activated to extinguish it.

This system is ideal for demonstrating basic principles of computer vision, robotics, and embedded systems in a practical, safety-oriented application.

---

## Features

* **Real-time Fire Detection:** Utilizes a custom-trained YOLOv5 model for accurate and fast fire detection from a live webcam feed.
* **Automated Pan-Tilt Mechanism:** Controls two servo motors to adjust the camera's (and attached nozzle's) horizontal (pan) and vertical (tilt) orientation to track detected fires.
* **Precision Targeting:** Continuously adjusts servo angles to center the detected fire within the camera's field of view.
* **Automated Water Suppression:** Activates a water pump (via a relay) when the detected fire is sufficiently centered, ensuring precise targeting of the suppression agent.
* **Modular Design:** Separated Python script for vision and control logic, and Arduino sketch for hardware control, allowing for easier development and debugging.

---

## Components Used

### Hardware
* **Arduino Board:** (e.g., Arduino Uno, Nano, or similar) - For controlling servos and pump relay.
* **Webcam:** Any USB webcam compatible with OpenCV.
* **SG90/MG996R Servos (x2):** One for pan (X-axis), one for tilt (Y-axis).
* **Pan-Tilt Bracket:** To mount the servos and webcam.
* **Water Pump:** A small DC water pump (e.g., mini submersible pump).
* **Relay Module:** Single-channel 5V relay module to switch the pump (e.g., KY-019).
* **Power Supply:** Appropriate power supply for the Arduino, servos, and pump.
* **Jumper Wires, Breadboard, etc.**

### Software
* **Python 3.x**
* **OpenCV:** For webcam input and frame processing.
* **PyTorch:** Deep learning framework for loading and running YOLOv5.
* **`serial` library (PySerial):** For serial communication between Python and Arduino.
* **`numpy`:** For numerical operations.
* **Arduino IDE:** For uploading the Arduino sketch.
* **YOLOv5:** Custom trained model (`yolov5s_best.pt`).

---

## Setup and Installation

### 1. Hardware Connections

Refer to the `media/connections.jpg` image for a visual guide on connecting your hardware components.

* **Arduino to Servos:**
    * **Servo X (Pan):** Signal pin to Arduino Digital Pin 9.
    * **Servo Y (Tilt):** Signal pin to Arduino Digital Pin 10.
    * Both servos' VCC to Arduino 5V (or external 5V regulated supply if using multiple/stronger servos).
    * Both servos' GND to Arduino GND.
* **Arduino to Pump Relay:**
    * Relay IN pin to Arduino Digital Pin 7.
    * Relay VCC to Arduino 5V.
    * Relay GND to Arduino GND.
* **Pump to Relay and Power Supply:**
    * Connect your external pump power supply (e.g., 6V/12V) positive to one side of the relay's **NO (Normally Open)** or **NC (Normally Closed)** terminal (depending on your relay and desired default state).
    * Connect the other side of the pump to the other terminal of the relay.
    * Connect the pump's negative terminal directly to the negative of its power supply.
    * **CRITICAL:** Ensure the pump's power supply ground is common with the Arduino's ground if they are not the same supply.

### 2. Arduino Setup

1.  **Install Arduino IDE:** Download and install from [arduino.cc/software](https://www.arduino.cc/software).
2.  **Install Servo Library:** The `Servo` library is usually pre-installed. If not, go to `Sketch > Include Library > Manage Libraries...` and search for "Servo".
3.  **Open Arduino Sketch:** Open `code/arduino_servo_pump.ino` in the Arduino IDE.
4.  **Configure `pumpPin` and `PUMP_ON_STATE` / `PUMP_OFF_STATE`:** In the `.ino` file, ensure `pumpPin` (default: 7) matches your connection. **Crucially, adjust `PUMP_ON_STATE` and `PUMP_OFF_STATE` if your relay is active-high instead of active-low.** (Most common small relays are active-low, meaning `LOW` signal turns them ON).
5.  **Select Board and Port:** Go to `Tools > Board` and select your Arduino board (e.g., "Arduino Uno"). Go to `Tools > Port` and select the serial port your Arduino is connected to.
6.  **Upload Sketch:** Click the "Upload" button (right arrow icon) to compile and upload the sketch to your Arduino.

### 3. Python Environment Setup

1.  **Clone this Repository (if you haven't already):**
    ```bash
    git clone https://github.com/neerajprao/Fire-Suppression-System
    cd your-repository-name
    ```
2.  **Create a Virtual Environment (Recommended):**
    ```bash
    python -m venv venv
    # On Windows:
    .\venv\Scripts\activate
    # On macOS/Linux:
    source venv/bin/activate
    ```
3.  **Install Dependencies:**
    ```bash
    pip install torch opencv-python pyserial numpy ultralytics
    ```
    * **Note on PyTorch:** For GPU support, you might need to install a specific PyTorch version. Refer to [pytorch.org](https://pytorch.org/get-started/locally/) for instructions based on your CUDA version. If running on CPU, the default `pip install torch` is usually sufficient.

### 4. Model and Configuration

1.  **YOLOv5 Model:** Ensure your `yolov5s_best.pt` model is placed in the `model/` directory.
2.  **Configure `fire_detection_system.py`:**
    * Open `code/fire_detection_system.py` in a text editor.
    * **`YOLOV5_MODEL_PATH`**: Make sure this points to your `.pt` file (e.g., `'model/yolov5s_best.pt'`).
    * **`WEBCAM_INDEX`**: Adjust if your webcam is not the default (0). Try 1, 2, etc., if 0 doesn't work.
    * **`SERVO_PORT`**: **CRITICAL!** Change this to your Arduino's serial port.
        * **Windows:** `COM3`, `COM4`, etc. (Check Device Manager).
        * **macOS:** `/dev/cu.usbserial-XXXX` or `/dev/tty.usbmodemXXXX` (Check `ls /dev/cu.*` or `ls /dev/tty.*`).
        * **Linux:** `/dev/ttyUSB0`, `/dev/ttyACM0`, etc. (Check `ls /dev/tty*` after plugging in Arduino).
    * **`BAUD_RATE`**: Must match the `Serial.begin()` value in your Arduino sketch (default is 9600).
    * **`ANGLE_ADJUSTMENT_SENSITIVITY_X` & `ANGLE_ADJUSTMENT_SENSITIVITY_Y`**: **TUNE THESE!** These values determine how aggressively the servos move. Adjust their magnitude and sign (positive/negative) until the camera correctly tracks the fire. A negative sign might be needed if the camera moves in the opposite direction.
    * **`PUMP_ACTIVATION_THRESHOLD_PIXELS`**: **TUNE THIS!** This value defines how close the fire needs to be to the frame center (in pixels) before the pump activates. A smaller value means more precise aiming is required.

---

## How to Run

1.  Ensure your Arduino is connected and the sketch is uploaded.
2.  Ensure your webcam is connected.
3.  Activate your Python virtual environment (if you created one).
4.  Navigate to the `code/` directory in your terminal:
    ```bash
    cd code/
    ```
5.  Run the Python script:
    ```bash
    python fire_detection_system.py
    ```

A window will open displaying the webcam feed with fire detections and servo/pump status overlays. The servos should start moving to track any detected fire, and the pump will activate once the fire is centered.

---

## Project Demonstration

Here are videos demonstrating the project in action:

* **Camera Output & Fire Detection:** [Link to your video demonstrating camera output and fire detection results]
* **Full Project Working (Hardware in action):** [Link to your video demonstrating the full hardware setup working, including servo movement and pump activation]

---

## Contributing

Contributions are welcome! If you have suggestions for improvements, bug fixes, or new features, please open an issue or submit a pull request.

---

## License

This project is open-source and available under the [MIT License](LICENSE) (or choose your preferred license).

---

## Contact

For any questions or inquiries, please feel free to reach out.

---