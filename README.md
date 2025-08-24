# Robotics Course Project: Automated Block Sorting with LeRobot SO-101 Arm

## Project Overview
This project challenges senior Mechanical Engineering students to design and implement an automated block sorting system using the LeRobot SO-101 robotic arm. The system will use a USB camera to detect the position of a wooden block on a table, then control the robot arm to pick up the block and place it in a designated area.

## Project Goals
- Use computer vision to detect and locate a wooden block on a workspace.
- Control the LeRobot SO-101 arm to pick up the block and place it in a target zone.
- Integrate hardware (robot arm, camera) and software (Python, OpenCV, serial communication).
- Demonstrate automation, feedback, and error handling in robotics.

## Detailed Setup
1. **Workspace Preparation**: Place the LeRobot SO-101 arm on a stable table. Mark the block placement area and the target zone with colored tape.
2. **Camera Mounting**: Mount a USB camera above the workspace to provide a clear view of the table, block, and arm.
3. **Block Placement**: Use a standard wooden block (e.g., 5x5x5 cm) as the object to be detected and moved.
4. **Robot Arm Connection**: Connect the LeRobot SO-101 arm to the control computer via USB/serial interface.
5. **Software Setup**: Install Python, OpenCV, and required libraries for camera and robot control.

## Materials Required
### LeRobot SO-ARM 101 Follower Arm
- 6 DOF: 5 for the arm, 1 for the gripper. Six motors: STS3215 12V, 1/345 gearing
- Price: $190, Ships from the US.
- Order: [SO-ARM101 Follower Only](https://partabot.com/products/so-arm101-follower-only?variant=43200383549555)
- Bill of Materials, CAD models: [SO-ARM100 GitHub](https://github.com/TheRobotStudio/SO-ARM100?tab=readme-ov-file)
- Assembly videos and documents: [LeRobot SO101 Docs](https://huggingface.co/docs/lerobot/so101)
- ROS simulation: [SO101 ROS Workspace](https://github.com/Pavankv92/lerobot_ws/tree/main)

### USB Camera
- 2MP USB Camera Module for SO-ARM100/101, 30fps, 3m cable
- Price: $28
- Order: [Wowrobo USB Camera](https://shop.wowrobo.com/products/2mp-usb-camera-module-for-so-arm100-101-30fps-3m-cable?utm_source=chatgpt.com)

## Sample Python Code
The SO-101 is a 6-DOF educational robot arm designed for automation and robotics research. It features Feetech STS 3215 bus servos for precise control and supports serial communication for easy integration with Python.
- Product page: https://www.lerobot.com/products/so-101-robot-arm
- Typical specs: 6-axis, payload up to 500g, USB/serial interface
- Order: https://www.lerobot.com/products/so-101-robot-arm


Recommended: Logitech C270 or similar USB webcam. Provides 720p video for object detection tasks.
- Product page: https://www.logitech.com/en-us/products/webcams/c270-hd-webcam.960-001063.html
- Order: https://www.amazon.com/Logitech-C270-Webcam/dp/B004YW7WCY/

### 1. Block Detection with USB Camera
Download the Feetech FD.1.9.8.3 bus servo debug software to test and debug servo motors:
- [FD.1.9.8.3.zip](./project_files/FD1.9.8.3.zip)
- Official page: http://www.feetechrc.com/software.html

```python
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define color range for wooden block (adjust as needed)
    lower = (10, 50, 50)
    upper = (30, 255, 255)
    mask = cv2.inRange(hsv, lower, upper)
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(frame, 'Block', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
    cv2.imshow('Block Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
```

### 2. Control LeRobot SO-101 Arm (Sample Serial Command)
```python
import serial
import time

# Connect to robot arm (adjust COM port as needed)
ser = serial.Serial('COM3', 115200, timeout=1)

def move_motor(motor_id, position):
    # Example command format for Feetech STS 3215 motor
    # Replace with actual protocol for SO-101
    cmd = bytearray([0xFF, 0xFF, motor_id, 0x05, 0x03, 0x1E, position & 0xFF, (position >> 8) & 0xFF])
    ser.write(cmd)
    time.sleep(0.1)

# Move motor 1 to position 512 (middle)
move_motor(1, 512)
ser.close()
```

## Deliverables
- Working Python scripts for block detection and robot arm control
- Documentation of setup and results
- Demonstration video

## Notes
- Adjust color detection parameters for your block and lighting.
- Refer to the SO-101 and Feetech STS 3215 manuals for command protocols.
- Ensure safety when operating the robot arm.
