import cv2
import mediapipe as mp
import numpy as np
import pyautogui
import time
import serial
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
import serial.tools.list_ports

# Disable PyAutoGUI fail-safe
pyautogui.FAILSAFE = False

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    model_complexity=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7,
    max_num_hands=1
)
mp_drawing = mp.solutions.drawing_utils

# Initialize webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Window and screen parameters
window_width, window_height = 640, 480
screen_width, screen_height = pyautogui.size()

# Smoothing and sensitivity parameters
smooth_factor = 0.85
base_sensitivity = 3.0
dead_zone = 0
cursor_x, cursor_y = screen_width // 4, screen_height // 4
prev_cursor_x, prev_cursor_y = None, None
cursor_locked = False
controls_enabled = True  # State for enabling/disabling scroll and volume
last_lock_toggle_time = 0
debounce_delay = 0.5  

# Click and drag parameters
click_threshold = 0.06
click_cooldown = 20
double_click_window = 0.2
drag_active = False
last_click_time = 0
click_frame_count = 0

# Speed adjustment parameters
slowdown_threshold = 0.1
min_speed_factor = 0.0

# Sensor midpoints for HC-SR04
VOLUME_MIDPOINT = 17.5  # cm
SCROLL_MIDPOINT = 17.5  # cm

# Function to auto-detect Arduino port
def detect_arduino_port():
    arduino_ports = []
    for port in serial.tools.list_ports.comports():
        if 'Arduino' in port.description or 'VID:PID' in port.hwid:
            arduino_ports.append(port.device)
    return arduino_ports

# Initialize serial connection to Arduino
def connect_arduino():
    print("Attempting to connect to Arduino...")
    arduino_ports = detect_arduino_port()
    if arduino_ports:
        print("Detected Arduino ports:", arduino_ports)
        for port in arduino_ports:
            try:
                arduino = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)  # Wait for connection to stabilize
                print(f"Successfully connected to Arduino on {port}.")
                return arduino
            except serial.SerialException as e:
                print(f"Failed to connect on {port}: {e}")
    
    # Fallback to manual selection
    while True:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("No serial ports detected. Please connect your Arduino and try again.")
            retry = input("Retry? (y/n): ").lower()
            if retry != 'y':
                return None
            continue
        
        print("Available ports:", [port.device for port in ports])
        port = input("Enter Arduino COM port from the list above (e.g., COM4): ").strip()
        try:
            
            arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            print(f"Successfully connected to Arduino on {port}.")
            return arduino
        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
            print("Possible fixes:")
            print("1. Run this script as Administrator (right-click -> Run as Administrator).")
            print("2. Check if another program (e.g., Arduino IDE) is using the port and close it.")
            print("3. In Device Manager, disable then re-enable the port under 'Ports (COM & LPT)'.")
            print("4. Restart your computer to release any locked ports.")
            print("5. Use a different USB port to assign a new COM number.")
            retry = input("Retry with a different port? (y/n): ").lower()
            if retry != 'y':
                return None

arduino = connect_arduino()
if not arduino:
    print("Failed to establish Arduino connection. Exiting.")
    exit()

# Initialize volume control
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))

# Main loop
while True:
    try:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Resize and flip frame
        frame = cv2.resize(frame, (window_width, window_height))
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process frame with MediaPipe
        frame_rgb.flags.writeable = False
        results = hands.process(frame_rgb)
        frame_rgb.flags.writeable = True

        # Handle cooldown
        if click_frame_count > 0:
            click_frame_count -= 1

        # Analyze hand landmarks only if cursor is not locked
        if results.multi_hand_landmarks and not cursor_locked:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(0, 255, 0)),
                    mp_drawing.DrawingSpec(color=(255, 0, 0))
                )

                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]

                thumb_x = thumb_tip.x * window_width
                thumb_y = thumb_tip.y * window_height
                target_x = np.interp(thumb_x, [0, window_width/2], [0, screen_width/2]) * base_sensitivity
                target_y = np.interp(thumb_y, [0, window_height/2], [0, screen_height/2]) * base_sensitivity

                if prev_cursor_x is None or prev_cursor_y is None:
                    if abs(thumb_x - window_width // 2) < 50 and abs(thumb_y - window_height // 2) < 50:
                        cursor_x, cursor_y = screen_width // 2, screen_height // 2
                    else:
                        cursor_x, cursor_y = target_x, target_y
                    prev_cursor_x, prev_cursor_y = cursor_x, cursor_y

                thumb_index_dist = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
                speed_factor = max(min_speed_factor, thumb_index_dist / slowdown_threshold) if thumb_index_dist < slowdown_threshold else 1.0

                new_x = cursor_x + (smooth_factor * (target_x - cursor_x) * speed_factor)
                new_y = cursor_y + (smooth_factor * (target_y - cursor_y) * speed_factor)
                if abs(new_x - cursor_x) > dead_zone or abs(new_y - cursor_y) > dead_zone:
                    cursor_x, cursor_y = new_x, new_y
                    prev_cursor_x, prev_cursor_y = cursor_x, cursor_y

                cursor_x = max(0, min(cursor_x, screen_width - 1))
                cursor_y = max(0, min(cursor_y, screen_height - 1))
                pyautogui.moveTo(int(cursor_x), int(cursor_y))

                cv2.circle(frame, (int(thumb_x), int(thumb_y)), 5, (0, 255, 255), -1)

                thumb_middle_dist = np.sqrt((thumb_tip.x - middle_tip.x)**2 + (thumb_tip.y - middle_tip.y)**2)
                thumb_ring_dist = np.sqrt((thumb_tip.x - ring_tip.x)**2 + (thumb_tip.y - ring_tip.y)**2)
                current_time = time.time()

                if thumb_index_dist < click_threshold and click_frame_count == 0:
                    if current_time - last_click_time < double_click_window:
                        pyautogui.doubleClick()
                        print("Double-click detected (Hand)")
                        cv2.putText(frame, "Double Click", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        arduino.write(b"LEFT_CLICK\n")
                    else:
                        pyautogui.click()
                        print("Left-click detected (Hand)")
                        cv2.putText(frame, "Left Click", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        arduino.write(b"LEFT_CLICK\n")
                    last_click_time = current_time
                    click_frame_count = click_cooldown

                if thumb_middle_dist < click_threshold and click_frame_count == 0:
                    pyautogui.rightClick()
                    print("Right-click detected (Hand)")
                    cv2.putText(frame, "Right Click", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    arduino.write(b"RIGHT_CLICK\n")
                    click_frame_count = click_cooldown

                # Drag functionality (Thumb + Ring)
                if thumb_ring_dist < click_threshold and not drag_active:
                    pyautogui.mouseDown()
                    drag_active = True
                    print("Drag started (Hand)")
                    arduino.write(b"DRAG_START\n")
                elif drag_active and thumb_ring_dist > click_threshold * 1.5:
                    pyautogui.mouseUp()
                    drag_active = False
                    print("Drag released (Hand)")
                    arduino.write(b"DRAG_END\n")

        # If cursor is locked, release any active drag
        if cursor_locked and drag_active:
            pyautogui.mouseUp()
            drag_active = False
            print("Drag released due to lock")

        # Display lock status
        if cursor_locked:
            cv2.putText(frame, "Cursor Locked", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(frame, "Cursor Unlocked", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display controls status
        cv2.putText(frame, f"Scroll/Volume: {'On' if controls_enabled else 'Off'}", 
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if controls_enabled else (0, 0, 255), 2)

        # Check for Arduino input
        if arduino.in_waiting > 0:
            arduino_data = arduino.readline().decode().strip()
            current_time = time.time()

            # Toggle cursor lock
            if "LOCK:" in arduino_data:
                lock_state = int(arduino_data.split("LOCK:")[1])
                if current_time - last_lock_toggle_time > debounce_delay:
                    cursor_locked = bool(lock_state)
                    last_lock_toggle_time = current_time
                    print(f"Cursor lock toggled via IR sensor: {'Locked' if cursor_locked else 'Unlocked'}")
                    cv2.putText(frame, f"Lock Toggled (IR): {'Locked' if cursor_locked else 'Unlocked'}", 
                                (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Toggle scroll and volume controls
            elif "CONTROLS:" in arduino_data:
                control_state = int(arduino_data.split("CONTROLS:")[1])
                if current_time - last_lock_toggle_time > debounce_delay:
                    controls_enabled = bool(control_state)
                    last_lock_toggle_time = current_time
                    print(f"Scroll/Volume toggled via Arduino: {'On' if controls_enabled else 'Off'}")
                    cv2.putText(frame, f"Controls Toggled: {'On' if controls_enabled else 'Off'}", 
                                (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Volume control
            elif "DIST1:" in arduino_data and controls_enabled and not cursor_locked:
                dist1 = int(arduino_data.split("DIST1:")[1])
                print(f"Distance 1 from HC-SR04 (Volume): {dist1} cm")
                if dist1 < VOLUME_MIDPOINT:
                    vol = max(0.0, min(1.0, (VOLUME_MIDPOINT - (dist1 - 5)) / (VOLUME_MIDPOINT - 5)))
                else:
                    vol = max(0.0, min(1.0, (30 - dist1) / (30 - VOLUME_MIDPOINT)))
                volume.SetMasterVolumeLevelScalar(vol, None)

            # Scroll control
            elif "DIST2:" in arduino_data and controls_enabled and not cursor_locked:
                dist2 = int(arduino_data.split("DIST2:")[1])
                print(f"Distance 2 from HC-SR04 (Scroll): {dist2} cm")
                scroll_speed = int(max(1, min(20, abs(30 - dist2) / 1.25)))
                if dist2 < SCROLL_MIDPOINT:
                    pyautogui.scroll(scroll_speed)
                elif dist2 > SCROLL_MIDPOINT:
                    pyautogui.scroll(-scroll_speed)

        # Add instructions
        instructions = (
            "Thumb: Move | Thumb+Index: Click/Double | Thumb+Middle: Right | "
            "Thumb+Ring: Drag | IR: Toggle Lock | Arduino: Toggle Scroll/Volume | HC-SR04 1: Volume | HC-SR04 2: Scroll"
        )
        cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Display the frame
        cv2.imshow('Enhanced Mouse Controller', frame)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error occurred: {e}")
        break

# Cleanup
cursor_x, cursor_y = screen_width // 2, screen_height // 2
pyautogui.moveTo(cursor_x, cursor_y)
cap.release()
cv2.destroyAllWindows()
hands.close()
if arduino:
    arduino.close()
    print("Arduino connection closed.")
