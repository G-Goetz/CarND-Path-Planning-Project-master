# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
The Term3 Simulator which contains the Path Planning Project can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  


### Introduction to this project work
In this project the goal is to navigate a vehicle in simulated highway traffic around an approximate 4 1/3 mile loop. A speed of around 50 MPH should be maintained whenever possible and lane changes made when necessary. 
Additionally the following constraints have to be fullfilled:
- The car should not go over 50 MPH.
- Maximum acceleration of 10 m/s^2
- Maximum jerk of 10 m/s^3.
- The car should only be outside of a lane for a few seconds when making lane changes.
- The car should not collide with any other vehicles on the road.


### Files
the following files are new or modified from the standard repository:

- main.cpp
- drive.h

the following file is downloaded from http://kluge.in-chemnitz.de/opensource/spline/

- spline.h

the folllowing files are original from the standard repository:

- helpers.h
- json.hpp
- Eigen-3.3 folder


### Vehicle movement
The movement of the vehicle is taken care of in main.cpp.
The basic implementation is carried over from https://github.com/csharpseattle and modified to my needs.
The implementation uses either the current vehicle position or waypoints from the previous iteration as
initial points and calculated points at 60 meters in front and on the designated lane.
With these points a spline is calculated using the spline.h library.
Waypoints along this spline are set based on the current speed and necessary acceleration to reach dhe intended speed.
Due to initial trouble in accelerating the vehicle in a proper way I had to modify the moriginal code so that
the ego vehicle speed is taken from previous iteration and not from the sensor data.

### Processing Sensor Fusion Data
The function "surrounding_vehicles_new" which is located in the drive.h and called from main.cpp is used to
read the targets around the ego vehicle, asign them to lanes and order them.
The result is written to a 3-dimensional vector: 3 lanes x objects per lane, ordered by distance x (distance, speed).
where each lane contains at 1 target behind and at least 1 target in front of the vehicle. if there are no actual
targets these are dummies that are far enough away and have speeds with no impact on our ego vehicle.
Having always the closest target behind on position 0 and the first target in front on position 1 allows an
easier and more efficient handling of the objects.

### Path Planning
The function "decide_action" which is located in the drive.h and called from main.cpp is used to
decide the lane to move to and decides the speed.
At first we decide the speed, this is always based on the actual lane and the intended lane.
The function for this is "getspeed" which considers distance and speed of the targets in front.
Since this is before the actual decission about the next intended lane this based on previous decissions.
Setting the speed according to the next lane directly after the decission caused crashes and unintended dangerous behavior.
Next we allocate weights to the lanes, the intention is that the weight is always > 0 and the lane with the least weight is chosen.
The following factors influence the weight:

- Speed of the targets on the lane in front
- Distance of the targets in front
- Number of targets in front
- Is this lane the fastest lane
- Is this lane the middle lane (slightly preferred)

We then order the lanes based on their weigth and check from lowest to highest weight if we can lanechange to them.
If the lane is two lanes away, instead we check if it is possible to the next lane (no direct double lanechanges allowed)
To check if a lanechange is possible the function "check_gap" is used.
The results, desired_lane and desired_speed, are written to global variables.



### Possible improvements
- Desired speed is currently only set only once each iteration and then is set for all waypoints,
  better would be to implement an adaption as soon as lane is changed.
- Ego vehicle should close distance to target in front in case it can then do a lanechange.
- Ego vehicle should brake if opening big enough to do a double lanechange behind the target on the side.
- Ego vehicle cannot properly react to lanechanges of other vehicles - simulator does this too rarely to study this case.
- Hard accelerating / braking again of targets sometimes causes issues.
- If a scenario occurs where all 3 lanes are blocked by equaly slow cars the behavior needs to be improved. 
- Performance with downloaded workspace and simulator is worse than online workspace, has to be improved
  The ego vehicle doesn't stay in lane very well but drifts.

