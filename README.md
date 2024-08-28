# AI-in-Mobile-Robotics
The goal of this project is to build an algorithm to distinguish two different object: a sphere and a cube. The robot must go towards the sphere and ignore the cube. The two objects (or more) can be also moved during the run of the algorithm.

## Algorithm
The shape identification is performed using two RANSAC regressor, one for the sphere , one for the cube.

## Trial
The most significant issue we encountered was to precisely distinguish the two different object while they have the same size. (diameter of sphere = side of the cube)
So we have to keep the two objects with different sizes (in our case, sphere bigger than the cube).
In this manner we managed to recognize and distinguish correctly the two shapes.

## To start
You need to replace the base "follow.py" from the lab sessions with our "follow_shape.py".
Then build the workspace with "colcon build".
Furthermore, you have to add a sphere object in the unity project, copiyng the cube and changing the shape of it.
In the end, run the same commands you would use for the "follow.py" execution.
