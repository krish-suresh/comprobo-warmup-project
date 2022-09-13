# Computational Robotics Warm-up Project
## Project Overiew
The purpose of this project was to develop an introductory understanding of ROS (Robot Operating System), understand how sensor data can be used to dynamically change robot behavior, and develop intuition on how to debug robotic systems using a [Neato](https://neatorobotics.com/). The Neato has a wide variety of features, including odometry tracking, 360 degree LiDAR, and bump sensors, which were implemented to develop numerous robot behaviors outlined in this document.

## Robot Behaviors
### Teleop
#### Behavior Description
Teleop (teloperation) allows for controlling the Neato from a remote location. In this implementation, the robot can be controlled through an externally connected laptop by interfacing with a Raspberry Pi attached to the Neato. 

The Teleop behavior maps 9 keyboard inputs (Q, W, E, A, S, D, Z, X, C) to 9 different drive commands. The specific mapping is shown below:
- Q: Veer Right (forward)
- W: Straight
- E: Veer left (forward)
- A: Turn Right
- S: Stop
- D: Turn Left
- Z: Veer Left (reverse)
- X: Reverse
- C: Veer Right (reverse)

In addition, pressing Ctrl+C stops the robot's motion and terminates execution of the Teleop node.
#### Features
The code structure for the Teleop Node consists of three primary functions: the initializer, get_key(), and run_loop(). 

The initializer is responsible for creating a publisher to publish Twist objects to the /cmd_vel topic, which dictates the linear and angular velocity of the Neato. It also creates a 10 Hz timer that calls run_loop() on every timer increment.

get_key() is what allows the node to interface with user input. The function listens to the user's keyboard and returns a string representation of the key that was pressed.

run_loop() is responsible for publishing Twists to the /cmd_vel topic. The function starts by calling get_key() to receive 
#### Limitations
The current implementation of Teleop has three main limitations: keypresses cannot be combined and the driving structure is not intuitive. 
limitations: can't combine key presses, hard to make small adjustments, loop


### Wall Follower

For this behavior, we were tasked with making a controller that makes the Neato follow a wall at a specific distance without colliding into it. The primary sensor we used was the spinning LIDAR attached on top of the Neato which provides distance measurements at all angles around the robot. To start, we identified two main factors that are important to control to ensure the Neato does not collide as it drives parallel to the wall: angle relative to the wall and normal distance to the wall. While we realize that only controlling the distance to the wall would also result in a wall following behavior, we wanted to penalize the controller from taking drastic angle changes and gradually respond to error. To find the distance normal to the wall from the Neato, we realized that the minimum of the lidar measurements would give the approximate distance since the smallest distance measurement will be closest to orthogonal. Next, for the angle measurement, we used the 45+ and 45- degree measurements from the right side of the Lidar as shown in the diagram below:

<img src="images/wall_follow.jpg"  width="300">

To calculate the angle relative to the wall by taking the inverse tangent of the right triangle formed by the LIDAR measurements and the wall. With both these values, we now had to control these values and minimize the angle `theta_wall` as well as `TARGET_DIST-normal_dist_to_wall`. To do so, we used a proportional controller with one coefficient for each parameter and tuned the values to allow for a rotational correction to be applied as the Neato is driving forward. 
```
TODO add stuff about detecting a wall in front of us.
TODO add stuff about not detecting walls
TODO improvements after testing on real robot
TODO code samples
```