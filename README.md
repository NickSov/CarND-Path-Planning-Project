# Path Planning Project

## Model Overview

At the center of this project is a simple finite state machine which governs the behavior of the vehicle. It manages when the vehicle should slow down, speed up, and change lanes. The vehicle does three main tasks:

- It moves as fast as legally possible (~49.5 mph) whenever it can, i.e. no obstacles in the way
- It slows down/speeds up in order to stay within a certain distance of obstacle, if one should be encountered
- After some time (7 seconds) after it follows the object, it considers a lane change
- If the lane change seems safe, e.g. no collision will occur, the vehicle commences with a lane change maneuver.

The finite state machine is shown in the chart below:

<img src="https://github.com/NickSov/CarND-Path-Planning-Project/blob/master/images/flow_path_plan.png" width="400">


## Reflection on Path Generation

Path generation was one of the main objectives of this project. For this project, path generation was split into two components:

1. Determine what decision (path) to take
2. How to generate a trajectory to reasonably follow the path

The first piece is explained in the Model Overview section above. The general decision making (behavior) and portions of trajectory generation were blended in this project into part 1. This is because finite state machines were used in order to determine whether a A) lane change is safe and B) whether to actually make the change. Also, a very simple state machine was used to either accelerate or decelerate the vehicle in order to keep a certain distance from the vehicle ahead. In a more complicated implementation, the decision making would include cost functions and generation of many possible options which the vehicle can take. Also, the trajectory generation wouldn't be blended into behavior, but would instead be separate and account only for determining the smoothest, safest possible trajectory.

Part 2 above is more traditional trajectory generation, in which comfort was accounted for when generating a lane change or slowing down/speeding up the vehicle. For this portion, I used cubic spline interpolation library by Timo Kluge was used. The library is located at: (https://kluge.in-chemnitz.de/opensource/spline/). I experimented with the cubic library by creating some sample lane change points in Fernet coordinates and plotting them. The plotting of the various lane change scenarios allowed me to visualize how the spline library worked. For example, below is a plot I created of a lane change from *Lane 6* to *Lane 2*.

<img src="https://github.com/NickSov/CarND-Path-Planning-Project/blob/master/images/spline_plot.png" width="280">

I noticed that varying the s-distance at the intersection of when the lane change occurs, made a substantial difference in regards to how smooth the lane change was. This allows for more time/distance to pass between the change over, hence smoothing out the path.
