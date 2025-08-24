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
The SO-101 is a 6-DOF educational robot arm designed for automation and robotics research. It features Feetech STS 3215 bus servos for precise control and supports serial communication for easy integration with Python.
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

### Feetech bus servo debug software
Download the Feetech FD.1.9.8.3 bus servo debug software to test and debug servo motors:
- [FD.1.9.8.3.zip](./project_files/FD1.9.8.3.zip)
- Official page: http://www.feetechrc.com/software.html

## Project Tasks

### Task 1: Pick and Place Wooden Blocks
**Goal:**
- Pick and place a variety of objects from one position to another
- Pick and place as many objects as possible in a given time window (about 90-120 seconds)
- The robot should be pre-programmed to conduct all tasks. Once it is set, no one can touch the robot or the object.

**Scoring:**
- A pick/place task is successful if the object is placed in the designated container box.
- Each successful pick/place task is counted 10 points
- The total score is calculated as the total number of tasks times 10 points

### Task 2: Pick a Marker Pen and Trace a Curve
**Goal:**
- (20 points) Robot grasps a marker pen (see dimensions from the previous slide) from a pen holder
- (40 points) Robot moves the pen tip to trace a curve (e.g. circle) on a field paper (letter size; exact dimensions will be given)
- (20 points) Robot places the marker back into the pen holder

**Total time:** 120 secs
**Maximum points:** 80

### Task 3: Random Placed Objects (Computer Vision)
**Goal:**
- Robot detects the position of a wooden block that is randomly placed on the field
- Robot grasps the detected block and places it into a designated area.

**Total time:** 120 secs
**Maximum points:** 80
- Field setup: same as in Task 1 and 2.

## Deliverables
- Working Python scripts for block detection and robot arm control
- Documentation of setup and results
- Demonstration videos
- A written paper describing the analysis, simulation and task accomplishment

## Notes
- Adjust color detection parameters for your block and lighting.
- Refer to the SO-101 and Feetech STS 3215 manuals for command protocols.
- Ensure safety when operating the robot arm.
