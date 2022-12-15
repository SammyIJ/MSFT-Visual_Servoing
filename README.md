# MSFT-Visual_Servoing
![UB](https://user-images.githubusercontent.com/62597513/145659645-9ab35c4d-694e-499d-8fad-6bf1091d32ec.jpeg)

## Prepared by:
### Boniface Samuel Ijeoma
### Misheck Emmanuel Chirwa

## Supervised by: 
   ###            Professor Omar Tahri
   
   
   
# Introduction
Visual servoing is a type of robot control that uses visual feedback from cameras to guide the movement of a robot. It involves using images from the cameras to determine the position and orientation of the robot's end effector in relation to a desired target, and then using that information to compute the necessary control signals for the robot to move to the target. This approach is useful in applications where the robot needs to manipulate objects in unstructured environments, or where it is not possible to use other methods of robot control, such as motion planning or force control.

# Project goal
The aim is to move a mobile robot "turtlebot3" from a current pose to a target pose using Aruco markers for pose detection with minimum distance using visual servoing techniques by calculating the pose of the camera.

 # Requirement 
- **Turtlebot3**: It is the newest generational mobile robot. It is a basic model to use AutoRace packages for autonomous driving on ROS. For this project, the TurtleBot3 Burger (with packages installed on it) was  provided by our university https://condorcet.u-bourgogne.fr/en.  The main components are shown below: 

![3](https://user-images.githubusercontent.com/62597513/145630186-4da6bcb0-b4aa-4c0d-b006-39453fabb56b.png)

source : https://www.robotis.us/turtlebot-3-burger-us/

- Camera - **Raspberry Pi ‘fish-eye’** camera
- The system was run through a **stationary PC**, connected to the TurtleBot3 running **Ubuntu 18.04**
- Robotic Operating System **(ROS)** software

# Work-Flow
- Camera Calibration
- Image Detection
- Pose Estimation
- Robot Navigation

# 1. Camera Calibration
Camera calibration is the process of determining the intrinsic and extrinsic parameters of a camera, which are necessary for the accurate mapping of pixel coordinates to real-world coordinates. The intrinsic parameters of a camera include the focal length and the principal point, which are specific to the camera itself. The extrinsic parameters, on the other hand, describe the position and orientation of the camera in relation to the world coordinate frame. Camera calibration is typically performed by capturing images of a known pattern, such as a checkerboard, from different viewpoints and using computer vision algorithms to extract the parameters from the images. These parameters are then used to correct for distortion and perspective effects in the images, enabling the accurate measurement of distances and sizes in the scene.

 To achieve the calibration, We used the ROS_camera_calibration_package for calibration of our monocular camera using a checkerboard calibration target. Therefore, camera calibration is an integral part of this project because it was understood that our camera has distortion which affects the functionality and sensining of the image view. For this reason, we calibrated the camera using a checkerboard to obtain the camera intrinsic and extrinsic parameters with distortion coefficients.
