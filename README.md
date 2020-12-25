# Robot_Kinematics

## Description
The code defines a function called IKinBodyIterates based on the function IKinBody from modern_robotics package (refer [here](https://github.com/NxRLab/ModernRobotics/blob/5e0f9e503cedb37fd9e5db706102b3b3fc288c22/packages/Python/modern_robotics/core.py#L699))
The function computes inverse kinematics in the body frame for an open chain robot and prints out a report for each iteration 
of the Newton-Raphson process, from initial guess to the final solution. Each iteration lists joint vector, SE3 end-effector configuration, error twist, angular error magnitude and linear error magnitude.

## Motivation
This script is a submitted peer-revewied assignment for Modern Robotics Course2: Robot Kinematics. 

## Problem Details
Configuration of the robot: UR5 6 arm robot  
![alt text](https://github.com/Suriya-Arulselvan/Robot_Kinematics/blob/main/UR5%206R%20robot.png)







## Reference
1. [Modern Robotics Course 2: Robot Kinematics](https://www.coursera.org/learn/modernrobotics-course2?specialization=modernrobotics)
2. [Modern Robotics Package](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python)
3. Modern Robotics: Mechanics, Planning, and Control, Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017, [ISBN 9781107156302](https://www.cambridge.org/us/academic/subjects/computer-science/computer-graphics-image-processing-and-robotics/modern-robotics-mechanics-planning-and-control?utm_source=SM&utm_medium=social&utm_campaign=9781107156302&utm_term=LFA) (downloadable [here](http://hades.mech.northwestern.edu/index.php/Modern_Robotics#Book))

