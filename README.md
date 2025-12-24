# Robot Lab 4 – Target Chasing and Avoidance

**Team Members:** Alex Anderson, Trenton Pham, Rafael Copado  
**Course:** CS 425 Robotics

This project implements an autonomous robot that combines vision-based target tracking with dynamic obstacle avoidance using OpenCV and a finite state machine.

---

## Project Overview

In this lab, we extended our previous work from vision-based object detection to build a robot capable of:

- Tracking and following a green beach ball
- Detecting and avoiding colored obstacles (cones)
- Reacting dynamically to target position and obstacles
- Integrating perception with motion control using a finite state machine

We implemented both:
- **Option 1:** Navigate between cones while chasing a ball  
- **Option 5:** Chase a ball while avoiding cones  

We also added enhanced color filtering and real-time HSV adjustment to improve detection performance.

---

## System Design

### Vision Processing
- Convert frames from the robot’s camera to **HSV color space**
- Apply color thresholds (`cv2.inRange`) to detect:
  - **Green** (target)
  - **Red & Yellow** (obstacles)
- Use morphological operations (*erode + dilate*) to reduce noise and improve mask quality

### Object Identification
- Use `cv2.connectedComponentsWithStats()` to find blobs and centroids
- Select the largest valid blob per color to represent the target or obstacles

### State Machine Control
The robot uses a finite state machine with states like:

- **SEARCHING** – Look for the green ball
- **TRACKING** – Move toward the ball
- **AVOID_LEFT / AVOID_RIGHT** – Turn to avoid red and yellow cones
- **STOP/ADJUST** – Center and drive straight when appropriate

This modular design keeps control logic organized and responsive.

---

## Results

The final implementation consistently:

- Tracked and followed the green beach ball
- Avoided red cones by turning left
- Avoided yellow cones by turning right
- Reacquired the target when temporarily lost

Morphological filtering greatly improved detection stability and reduced false positives due to lighting and background effects.

---

## Demo Video

🎥 **Watch the autonomous robot in action:**  
https://youtu.be/epj4aet0wzc?si=4TWz7I34MRizFc92

*(This video demonstrates the robot successfully navigating obstacles while pursuing the green ball.)*

---

## Reflection

This lab deepened our understanding of:

- **OpenCV color filtering and morphology**
- **Connected components for object detection**
- **Finite state machine design for robot behavior**

The main challenge was tuning HSV thresholds for different lighting conditions. Once the vision pipeline was stable, the robot’s behavior was predictable and robust.

This project illustrates how relatively simple vision techniques can be combined with structured control logic to achieve autonomous operation.

---

## Future Improvements

If given more time, we would:

- Implement adaptive thresholding for lighting changes
- Merge cone tracking with line following for advanced navigation
- Improve drive-straight control to reduce unnecessary searching
- Enhance multi-object discrimination for complex environments

---

## Files in this Repository

- `lab4Robot.py` — Main autonomous control and vision code
