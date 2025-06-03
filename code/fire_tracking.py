import torch
import cv2
import numpy as np
import serial
import time

# --- Configuration ---
YOLOV5_MODEL_PATH = 'yolov5s_best.pt'                     # <--- IMPORTANT: Update this to your .pt model file path
WEBCAM_INDEX = 0                                          # <--- Adjust if you have multiple webcams (0 is usually default)
SERVO_PORT = '/dev/cu.usbserial-140'                      # <--- IMPORTANT: Update this to your Arduino's serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux/macOS)
BAUD_RATE = 9600                                          # <--- Must match the baud rate set in your Arduino code

# X-axis (Pan) Servo Configuration
SERVO_X_CENTER_ANGLE = 90                                 # Initial center angle for X-axis servo (adjust to physically center camera)
SERVO_X_MAX_ANGLE = 180                                   # Maximum X-axis servo angle
SERVO_X_MIN_ANGLE = 0                                     # Minimum X-axis servo angle
# TUNE THIS: How much X-axis angle changes per pixel offset.
# Positive value: camera pans right for fire to the right.
# Negative value: camera pans left for fire to the right.
ANGLE_ADJUSTMENT_SENSITIVITY_X = -0.03                    # Adjust sign if direction is wrong (e.g., -0.05)

# Y-axis (Tilt) Servo Configuration
SERVO_Y_CENTER_ANGLE = 90                                 # Initial center angle for Y-axis servo (adjust to physically center camera)
SERVO_Y_MAX_ANGLE = 180                                   # Maximum Y-axis servo angle
SERVO_Y_MIN_ANGLE = 0                                     # Minimum Y-axis servo angle
# TUNE THIS: How much Y-axis angle changes per pixel offset.
# Positive value: camera tilts down for fire below.
# Negative value: camera tilts up for fire below.
ANGLE_ADJUSTMENT_SENSITIVITY_Y = 0.03                     # Adjust sign if direction is wrong (e.g., -0.05)

CONFIDENCE_THRESHOLD = 0.2                                # Minimum confidence score for fire detection
FRAME_WIDTH = 640                                         # Desired frame width for processing
FRAME_HEIGHT = 480                                        # Desired frame height for processing
FIRE_CLASS_NAME = 'fire'                                  # <--- IMPORTANT: Ensure this matches your model's exact fire class name (e.g., 'fire', 'Fire', 'flame')

# --- Pump Control Configuration ---
# <--- IMPORTANT: TUNE THIS VALUE ---
# This is the maximum allowed Euclidean pixel distance from the frame center
# for the pump to activate. Lower value = fire must be more precisely centered.
# Calculate as Euclidean distance: sqrt(offset_x^2 + offset_y^2)
PUMP_ACTIVATION_THRESHOLD_PIXELS = 50 # Example: activates if fire is within 50 pixels of center in any direction
# If pump activates too easily, lower this. If it needs to be more precise, lower it.
# If it never activates, increase it for testing.
# --- END PUMP TUNE ---

# Global variables for tracking current servo angles and pump state
current_servo_angle_x = SERVO_X_CENTER_ANGLE
current_servo_angle_y = SERVO_Y_CENTER_ANGLE
current_pump_state = 0 # 0 for OFF, 1 for ON (matches Arduino's expectation)

# --- Pan-Tilt Servo and Pump Command Function ---
# This function MUST be defined BEFORE it is called anywhere in the script.
def send_pan_tilt_and_pump_state(angle_x, angle_y, pump_state):
    """Sends X, Y desired servo angles and pump state to the Arduino via serial."""
    global current_servo_angle_x
    global current_servo_angle_y
    global current_pump_state
    global ser

    if ser:
        # Constrain angles within their defined limits (0-180 for standard servos)
        angle_x = max(SERVO_X_MIN_ANGLE, min(SERVO_X_MAX_ANGLE, angle_x))
        angle_y = max(SERVO_Y_MIN_ANGLE, min(SERVO_Y_MAX_ANGLE, angle_y))

        # Ensure pump_state is strictly 0 or 1
        pump_state = 1 if pump_state else 0

        # Format message as "X_angle,Y_angle,Pump_State\n" for Arduino parsing
        message = f"{int(angle_x)},{int(angle_y)},{int(pump_state)}\n"
        try:
            ser.write(message.encode('utf-8'))
            current_servo_angle_x = angle_x
            current_servo_angle_y = angle_y
            current_pump_state = pump_state # Update global state after successful send
            # print(f"Sent: X={int(angle_x)}, Y={int(angle_y)}, Pump={pump_state}") # Verbose serial send logging
            time.sleep(0.02) # Small delay to allow Arduino to process. Increased slightly for stability.
        except serial.SerialException as e:
            print(f"Error sending data to Arduino: {e}")
            ser.close()
            ser = None # Mark as disconnected upon error, preventing further sends
    # else:
        # print(f"Command (X={int(angle_x)}, Y={int(angle_y)}, Pump={pump_state}) not sent: Serial not connected.")


# --- Initialize Serial Communication with Arduino ---
ser = None # Initialize ser as None globally
try:
    ser = serial.Serial(SERVO_PORT, BAUD_RATE, timeout=1)
    print(f"Attempting to connect to Arduino on {SERVO_PORT} at {BAUD_RATE} baud...")
    time.sleep(2) # Give Arduino some time to reset after connection
    print("Serial connection established.")
    # Send initial center angles and pump off command to ensure a known state
    send_pan_tilt_and_pump_state(SERVO_X_CENTER_ANGLE, SERVO_Y_CENTER_ANGLE, 0)
except serial.SerialException as e:
    print(f"Could not open serial port {SERVO_PORT}: {e}")
    print("Proceeding without servo/pump control. Please ensure Arduino is connected and port is correct.")
    ser = None # Explicitly set ser to None if connection fails

# --- Load YOLOv5 Model ---
try:
    # Ensure force_reload=False for local model loading efficiency
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLOV5_MODEL_PATH, force_reload=False)
    model.eval() # Set model to evaluation mode
    print(f"YOLOv5 model loaded successfully from {YOLOV5_MODEL_PATH}")
    print(f"Model Class Names: {model.names}") # Print class names for verification
except Exception as e:
    print(f"Error loading YOLOv5 model: {e}")
    print("Please ensure the model path is correct and PyTorch/YOLOv5 dependencies are met.")
    exit()

# --- Initialize Webcam ---
cap = cv2.VideoCapture(WEBCAM_INDEX)
if not cap.isOpened():
    print(f"Error: Could not open webcam at index {WEBCAM_INDEX}.")
    exit()

# Set desired frame resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
print(f"Webcam opened successfully. Frame resolution: {FRAME_WIDTH}x{FRAME_HEIGHT}")

# --- Main Loop ---
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Exiting...")
            break

        # Resize frame for consistent processing
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        # Perform inference - model expects RGB (OpenCV reads BGR, so convert with [..., ::-1])
        results = model(frame[..., ::-1])

        # Get detected bounding boxes and class information
        # results.xyxy[0] contains detections as numpy array [x1, y1, x2, y2, conf, cls]
        detections = results.xyxy[0].cpu().numpy()

        fire_detected = False
        pump_should_be_on = 0 # Default to OFF (0) for each frame

        # Initialize target center to frame center. If no fire, servos will recenter.
        target_x_center = FRAME_WIDTH // 2
        target_y_center = FRAME_HEIGHT // 2

        # Iterate through detections to find fire
        for *xyxy, conf, cls in detections:
            class_name = model.names[int(cls)]

            # Check for fire class and confidence threshold
            if class_name == FIRE_CLASS_NAME and conf > CONFIDENCE_THRESHOLD:
                fire_detected = True
                x1, y1, x2, y2 = map(int, xyxy)

                box_center_x = (x1 + x2) // 2
                box_center_y = (y1 + y2) // 2

                target_x_center = box_center_x # Update target to the fire's center
                target_y_center = box_center_y

                # Draw the bounding box for the detected fire
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) # Green box
                cv2.putText(frame, f'{class_name} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Draw a red circle at the center of the detected fire box
                cv2.circle(frame, (target_x_center, target_y_center), 5, (0, 0, 255), -1)

                break # Process only the first (presumably largest/highest confidence) detected fire for control

        # --- Servo and Pump Control Logic ---
        frame_center_x = FRAME_WIDTH // 2
        frame_center_y = FRAME_HEIGHT // 2

        # Draw a blue circle at the center of the frame for visual reference of camera aim
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1) # Blue dot at frame center

        if fire_detected:
            offset_x = target_x_center - frame_center_x
            offset_y = target_y_center - frame_center_y

            # Calculate the Euclidean distance of the target from the frame center
            distance_from_center = np.sqrt(offset_x**2 + offset_y**2)

            # Determine X-axis servo adjustment
            # Logic: If fire is to the right (positive offset_x), we want to turn camera right.
            # If your servo moves right with INCREASING angle, use +offset_x.
            # If your servo moves right with DECREASING angle, use -offset_x.
            angle_adjustment_x = offset_x * ANGLE_ADJUSTMENT_SENSITIVITY_X # Adjust sign here if needed

            # Determine Y-axis servo adjustment
            # Logic: If fire is below (positive offset_y), we want to tilt camera down.
            # If your servo tilts down with INCREASING angle, use +offset_y.
            # If your servo tilts down with DECREASING angle, use -offset_y.
            angle_adjustment_y = offset_y * ANGLE_ADJUSTMENT_SENSITIVITY_Y # Adjust sign here if needed

            new_servo_angle_x = current_servo_angle_x + angle_adjustment_x
            new_servo_angle_y = current_servo_angle_y + angle_adjustment_y

            # --- Pump Activation Logic ---
            if distance_from_center < PUMP_ACTIVATION_THRESHOLD_PIXELS:
                pump_should_be_on = 1 # Turn pump ON
                cv2.putText(frame, 'PUMP: ACTIVATED!', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) # Red text
            else:
                pump_should_be_on = 0 # Keep pump OFF (fire is detected but too far from center)
                cv2.putText(frame, 'PUMP: OFF (Centering)', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2) # Yellow text

            # Send angles and pump state to Arduino
            send_pan_tilt_and_pump_state(new_servo_angle_x, new_servo_angle_y, pump_should_be_on)

            # Display servo/offset/pump info on frame for debugging
            cv2.putText(frame, f'Servo X: {int(current_servo_angle_x)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f'Servo Y: {int(current_servo_angle_y)}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f'Offset X: {offset_x}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f'Offset Y: {offset_y}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f'Dist from Center: {distance_from_center:.1f}', (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f'Pump State: {"ON" if current_pump_state else "OFF"}', (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255) if current_pump_state == 0 else (0, 0, 255), 2)


            # --- Debug Prints to Console (ENABLED by default for troubleshooting) ---
            print(f"\n--- Fire Detected ---")
            print(f"Box Center: ({target_x_center}, {target_y_center})")
            print(f"Offsets: X={offset_x}, Y={offset_y}, Dist={distance_from_center:.1f}")
            print(f"Current Servo Angles: X={current_servo_angle_x:.2f}, Y={current_servo_angle_y:.2f}")
            print(f"New Target Angles: X={new_servo_angle_x:.2f}, Y={new_servo_angle_y:.2f}")
            print(f"Pump Should Be On: {'ON' if pump_should_be_on else 'OFF'} (Threshold: {PUMP_ACTIVATION_THRESHOLD_PIXELS})")
            print(f"Actual Pump State Sent: {'ON' if current_pump_state else 'OFF'}")
            print("---------------------")
            # --- End Debug Prints (Comment out or remove these print() lines after tuning) ---

        else: # No fire detected in the current frame
            # Ensure pump is OFF and attempt to recenter servos
            cv2.putText(frame, 'No Fire Detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, 'PUMP: OFF', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f'Actual Pump State Sent: {"ON" if current_pump_state else "OFF"}', (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            send_pan_tilt_and_pump_state(SERVO_X_CENTER_ANGLE, SERVO_Y_CENTER_ANGLE, 0) # Send pump OFF (0)

            # Debug print for no fire (optional)
            # print(f"\n--- No Fire Detected ---")
            # print(f"Attempting to recenter servos and turn pump OFF.")
            # print(f"Actual Pump State Sent: {'ON' if current_pump_state else 'OFF'}")
            # print("------------------------")


        # Display the frame
        cv2.imshow('Fire Detection & Pan-Tilt Servo/Pump Control', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Script terminated by user via KeyboardInterrupt.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    # Release webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    # Close serial port if it was opened
    if ser:
        # Ensure pump is turned off and servos are centered before closing
        print("Sending final OFF command to pump and centering servos...")
        send_pan_tilt_and_pump_state(SERVO_X_CENTER_ANGLE, SERVO_Y_CENTER_ANGLE, 0)
        time.sleep(0.5) # Give Arduino a moment to process final command
        ser.close()
        print("Serial port closed.")
    print("Resources released. Exiting program.")