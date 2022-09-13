
## Wall Follower

For this behavior, we were tasked with making a controller that makes the Neato follow a wall at a specific distance without colliding into it. The primary sensor we used was the spinning LIDAR attached on top of the Neato which provides distance measurements at all angles around the robot. To start, we identified two main factors that are important to control to ensure the Neato does not collide as it drives parallel to the wall: angle relative to the wall and normal distance to the wall. While we realize that only controlling the distance to the wall would also result in a wall following behavior, we wanted to penalize the controller from taking drastic angle changes and gradually respond to error. To find the distance normal to the wall from the Neato, we realized that the minimum of the lidar measurements would give the approximate distance since the smallest distance measurement will be closest to orthogonal. Next, for the angle measurement, we used the 45+ and 45- degree measurements from the right side of the Lidar as shown in the diagram below:

<img src="images/wall_follow.jpg"  width="300">

To calculate the angle relative to the wall by taking the inverse tangent of the right triangle formed by the LIDAR measurements and the wall. With both these values, we now had to control these values and minimize the angle `theta_wall` as well as `TARGET_DIST-normal_dist_to_wall`. To do so, we used a proportional controller with one coefficient for each parameter and tuned the values to allow for a rotational correction to be applied as the Neato is driving forward. 
```
TODO add stuff about detecting a wall in front of us.
TODO add stuff about not detecting walls
TODO improvements after testing on real robot
TODO code samples
```