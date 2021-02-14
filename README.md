[//]: # (Image References)

[simulator]: ./output_images/simulator.png "Simulator"
[simulation]: ./output_images/simulation.gif "Simulation"
[flowchart]: ./output_images/flowchart.png "Flowchart"
[behavioral]: ./output_images/behavioral.png "Behavioral"
[highway]: ./output_images/highwayDriving.png "Highway"
[best]: ./output_images/best.png "Best"
[432]: ./output_images/432.png "432"

# **Path Planning** 

## Report

---

**Path Planning Project**
# Overview
This repository contains all the code needed to run the project for the Path Planning course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

In this project a path planning is implemented in C++.

![alttext][highway]

## Prerequisites

This project involves the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). 

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

Required tools are:
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
  
## Running the Code
This project involves the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.
```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
```
Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:
```sh
1. ./clean.sh
2. ./build.sh
3. ./run.sh
```
## Connecting with the simulator

Here is the data provided from the Simulator to the C++ Program.

#### Main car's localization Data (No Noise)
```
["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH
```
#### Previous path data given to the Planner
```
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value
```
#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
```
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 
```

To run the Path Planning after building and installing: either `./path_planning` or [`./run.sh`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/run.sh). The output will be as below:
```sh
Listening to port 4567
```
Here the Path Planning will be waiting for the simulator to launch and start, and once it started, the below message will appear in the console.
```sh
Connected!!!
```
Initially, the simulator looks like this

![alt text][simulator]

# Implementing the Path Planning
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   highway_map.csv
|   
|   
|___src
    |   helpers.h
    |   json.hpp
    |   spline.h
    |   main.cpp
    |   pathPlanning.h
    |___Eigen-3.3
```

## Inputs to the Path Planning
You can find the inputs to the path planning in the [`data`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/tree/master/data) directory.

#### The Map
[`highway_map.csv`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/data/highway_map.csv) includes a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Implementation Overview

The path planning consists of three major components which are prediction, behavioral planning, and trajectory planning.

The path planning is initialized in the [`main.cpp`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/src/main.cpp) by passing the map waypoints of the highway to the constructor.

![alttext][flowchart]


### Prediction
In the prediction part, the code works with telemetry and sensor fusion data. The goal of the prediction section is to study the surrounding environment and check the status of all the other vehicles which are driving around the ego-vehicle. What the prediction is trying to output is one or more of the following situations:

* A vehicle is driving in front of the ego-vehicle blocking the traffic and has a close distance from the ego-vehicle.
* A vehicle is driving to the right of the ego-vehicle or close from it, thus making a right lane change not safe.
* A vehicle is driving to the left of the ego-vehicle or close from it, thus making a left lane change not safe.

By calculating the lane of all the other vehicles presented in the environment and their position at the end of the last plan trajectory, it will be known if one or more of such cases is being hit. The safety distance between the ego-vehicle and other vehicles is set to 30.0 meters.

The prediction is implemented within the [`PathPlanning()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L11) class in the [`pathPlanning.h`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/src/pathPlanning.h) under the method [`prediction()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L70).

### Behavioral Planning
The behavioral planning part is simply tries to take an action based on the information it received from the prediction.
![alttext][behavioral]

The behavioral planning works as it gives always a high priority to drive in the middle lane. Below is a brief of how each action will be taken:
* Is there a car in front of the ego-vehicle blocking the traffic?
  * **Yes**: Is it safe to make a left lane change?
    * **Yes**: Make a left lane change
    * **No**: Is it safe to make a right lane change?
      * **Yes**: Make a right lane change
      * **No**: Keep in the lane and follow the vehicle ahead but deccelerate to maintain safety distance and avoid collision
  * **No**: Is the ego-vehicle on the middle lane?
    * **Yes**: Accelerate till the maximum allowed velocity
    * **No**: 
      * Is the ego-vehicle on the left lane?
        * **Yes**: Is it safe to make a right lane change?
          * **Yes**: Make a right lane change to the middle lane
          * **No**: Keep in the lane and accelerate till the maximum allowed velocity
      * Is the ego-vehicle on the right lane?
        * **Yes**: Is it safe to make a left lane change?
          * **Yes**: Make a left lane change to the middle lane
          * **No**: Keep in the lane and accelerate till the maximum allowed velocity

The behavioral planning is implemented within the [`PathPlanning()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L11) class in the [`pathPlanning.h`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/src/pathPlanning.h) under the method [`behavioral_planning()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L122).

### Trajectory Planning
In the trajectory planning, the behavioral output is used as well as the car coordinates and past path points. Afterwards, a calculation of the desired trajectory is done.

A smooth trajectory is calculated using the spline which contains information about previous path points of the  vehicle and some future points from the map. The actual future path points of the ego vehicle are derived from the spline. This will help in doing a smooth trajectory and avoids increasing the jerk.

In order to avoid sudden changes in the velocity, the distance between the points in the path are incrementally increased or decreased.

The trajectory planning is implemented within the [`PathPlanning()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L11) class in the [`pathPlanning.h`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/master/src/pathPlanning.h) under the method [`trajectory_planning()`](https://github.com/mhusseinsh/CarND-Path-Planning-Project/blob/a2a6492b8c72589071317152c6fea6d7cdaae98e/src/pathPlanning.h#L158).

## Results Evaluation and Success Criteria
Based on the defined [Rubric Points](https://review.udacity.com/#!/rubrics/1971/view) the Path Planning should achieve the below points:

1. **The car is able to drive at least 4.32 miles without incident.**: The vehicle was able to drive more than 4.32 miles without any incident, it also drove more than 15 miles.
   ![alttext][432]
   ![alttext][best]

2. **The car drives according to the speed limit.**: The car did not exceed the maximum allowed speed limit of 50 MPH (No red warnings appeared for exceeding the speed limit), also it did not drive slow and obstructed the traffic flow at all (except when it is slowing down because it was obstructed by traffic).

3. **Max Acceleration and Jerk are not Exceeded.**: The maximum acceleration and jerk were not exceeded (No red warning messages appeared as well).
   
4. **Car does not have collisions.**: Within all the driving tests, the car did not have any collisions.
   
5. **The car stays in its lane, except for the time between changing lanes.**: Based on the intenteded behavior, the car was always staying in its lane, except when doing a lane change due to a vehicle which is obstructing the traffic flow in front of it, or when it is on the left or right side, and it is safe to go back to the middle lane.
   
6. **The car is able to change lanes.**: The car was able to change lanes smoothly to left or right, when there is a vehicle in the front obstructing the traffic flow, or when it is returning back to the middle lane (if it is safe).

And here is a run example:

![alt text][simulation]

