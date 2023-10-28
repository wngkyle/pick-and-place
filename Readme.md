# Pick and Place

### Table of Contents

- [Introduction](#introduction)
- [Methodology](#methodology)
- [Algorithm](#algorithm)
- [Built With](#built-with)
- [Technologies Used](#technologies-used)

### Introduction

The objective of this project is to implement a controller that allows the robot to pick up the two red blocks on the table and drop them in the green bowl. The robot used in this project is the [Frank-Emika Panda robot](https://www.franka.de/), also known as the Panda Robot. It has 7 revolute joints, and its kinematics are shown in the figure below. 

<p align="center">
  <img src="https://github.com/wngkyle/pick-and-place/assets/99611120/5f01c31b-9ee9-4e09-85c7-6ec4cb7c6983">
</p>

<p align="center">
  <img src="https://github.com/wngkyle/pick-and-place/assets/99611120/c54bbe8f-0012-4e42-a032-df12d980fb0b">
</p>

### Methodology

The controller employed in this project is the resolve rate motion controller. Resolve rate control focuses in the end-effector space where the velocity of the end effector is computed based on its own position in the workspace. Thus, instead of calculating joint commands and generating trajectory in the joint space, all joint velocities will be computed in the end-effector space in this project.

The inverse Jacobian matrix is used to help convert the desired velocities and acceleration of the end effector into joint velocities of the robot. The inverse Jacobian matrix is a mathematical construct that describes the joints’ relations and robot’s kinematics with respect to a certain frame. This frame can be either the end effector frame or the spatial frame. The inverse Jacobian matrix used in this project will be working in the end-effector frame but oriented like the spatial frame. In doing so, we ensure the gripper is always oriented in the same direction even while the body of the robot may be moving or oriented in a different direction. 

The reason why the resolve rate motion controller is employed is that it produces a higher degree of accuracy and precision compared to joint space generation. In addition, the inverse kinematic function used in joint space generation can become really complicated and hard to derive in this project because we are using a robot with 7 revolute joints. The inverse kinematic function has already become relatively complex when the robot with 2 revolute joints is used. With a robot that has 7 revolute joints, the equations for inverse kinematics will be almost impossible to derive as we need to consider so many different possibilities. However, the resolve rate motion controller comes with some limitations. For instance, the resolve rate controller may not be ideal for tasks that require the robot to apply a specific force or interact with its environment in a more complex way. For tasks like these, an impedance controller might be a better option as it allows robots to behave like a spring. 

As for the path taken by the robot to complete the task, the robot navigates to a position above block 1 first from its initial position and slowly descends to where block 1 is on table level. The robot grip block 1 using its gripper and ascends back to a position above the block’s initial position. The robot moves to a position above the bowl and then drops the block into the bowl. The same concept applies to block 2. The robot moves from the top of the bowl to a position above block 2 and then descends to where block 2 is. The robot grips block 2 and ascends back to a position above the block’s initial position. The robot then moves to a position above the bowl and drops the block. Here block 1 refers to the block on the right and block 2 refers to the block on the left when viewed from the base frame. 

<p align="center">
  <img src="https://github.com/wngkyle/pick-and-place/assets/99611120/3f5ffcbd-514b-4a78-8280-5a9dc9d2c94b">
</p>

### Algorithm

<p align="center">
  <img src="https://github.com/wngkyle/pick-and-place/assets/99611120/c54bbe8f-0012-4e42-a032-df12d980fb0b">
</p>

### Built With

### Technologies Used
