# AI-in-Mobile-Robotics
The goal of this project is to build an algorithm to distinguish between two different object: a sphere and a cube, at first in a unity environment, then in a real world scenario.
The robot must go towards the sphere and ignore the cube and stop itself at about 20cm of distance.
The two objects (or more) can be also moved during the run of the algorithm.

## Requirements
\textbf{Unity} and \textbf{ROS2} are required, we suggest to follow this repository to install and configure the environment https://gitlab.com/TrottiFrancesco/mobile_robotics_lab

## Files usage
<b>project_exam_grid.unity</b> and <b>project_exam_grid.unity.meta</b>, are the the arena files configuration, containing the agent and the obstacles
<b>plan_and_exec.py</b> is the core program to run

## Algorithm
Shape identification is performed using two RANSAC regressor, one for the sphere, one for the cube.
Random sample consensus, RANSAC is an iterative algorithm for the robust estimation of parameters from a subset of inliers from the complete data set.
It estimates a mathematical model from a data set that contains outliers. The RANSAC algorithm works by identifying the outliers in a data set and estimating the desired model using data that does not contain outliers.

In our project, we used RANSAC to recognize spheres or cubes, fitting the Lidar datas on the two different models learned by the alghoritm

##### Linear regression without RANSAC
![alt text](https://pyihub.org/wp-content/uploads/2023/12/linear-regression-vs.png)
##### Linear regression with RANSAC
![alt text](https://pyihub.org/wp-content/uploads/2023/12/ransac-model.png)


## Trial
The most significant issue we encountered was to precisely distinguish the two different object while they have the same size. (diameter of sphere = side of the cube)
So we had to keep the two objects with different sizes (in our case, cube bigger than the sphere).
In this manner we managed to recognize and distinguish correctly the two shapes.

### How it works
![alt text](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExM2twdGhmMjUycWRndHlzam1nZGFlZGhzYnF2aWt5czA5YWhmNmE5ZyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/FYT7vHOQgI7f3yWSoN/giphy.gif)   
In the video above, we can see how the robot detects the sphere and follow it, then it stops at the right distance and starts rotating looking for a new sphere, ignoring the big cube next to it.

## To start
You need to replace the base "follow.py" from the lab sessions with our "follow_shape.py".
Then build the workspace with "colcon build".
Furthermore, you have to add a sphere object in the unity project, copying the cube and changing the shape of it.
In the end, run the same commands you would use for the "follow.py" execution.
