# task2_b0

This ROS 2 node estimates the global positions of detected objects based on camera detections during a yaw rotation. It is intended for use in autonomous search operations and outputs a rectangular region that can be used to perform a grid search around potential targets.

## Description

- The node starts when it receives the `TASK2B_0` flag and stops when it receives `OFF`.
- During its active period, the drone is expected to rotate around its yaw axis.
- The node computes the global positions of detected objects and returns **4 global coordinates** that form the corners of a minimal bounding rectangle.
- These 4 points are published to the `/pose_targets` topic and can be used by another module to conduct a grid search.

## How to Run

```bash
ros2 run task2_b0 search_pose_estimator_odom
```

Make sure all relevant topics are being published:

```/fmu/out/vehicle_odometry``` (PX4 odometry)

```/detection_result``` (camera centroid detections)

```/detector_flag ```(task control flag: expects 'TASK2B_0' to start and 'OFF' to stop)


## Output
Once the node receives the 'OFF' flag, it publishes a comma-separated list of the 4 global coordinates (x, y) as a string on:

```bash
/pose_targets
```
These represent the corners of the estimated rectangle containing the detected targets.
