# Vision-Guided Robotic Arm Navigation System

This project focuses on implementing a vision-guided robotic arm navigation system using ArUco markers for localization and control. 

## Workflow:
1. **Marker Detection**: ArUco markers are detected in a given workspace and their pixel coordinates are extracted from a camera feed.
2. **Coordinate Transformation**: The pixel coordinates are transformed into physical coordinates using calibration equations derived from camera parameters.
3. **Inverse Kinematics**: The physical positions, along with predefined orientations, are input to an inverse kinematics solver to compute the joint angles required for the robot's end effector to navigate to the specified positions.
4. **Validation**: The computed joint angles are validated in a MATLAB-based digital twin simulation.
5. **Control**: The validated joint angles are transmitted to a physical robotic arm via a TCP socket connection.

## Results:
The robotic arm successfully replicated the motion observed in the digital twin, navigating between marker positions with high accuracy.

## Conclusion:
This experiment demonstrates the integration of computer vision, robotics kinematics, and control systems to achieve precise and reliable robotic motion in both simulated and real-world environments.
