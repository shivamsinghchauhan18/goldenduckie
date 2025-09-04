# obstacle_tracking

Temporal tracker and smoother for obstacle detections.

- Subscribes: `~obslist_in` (ObstacleProjectedDetectionList), default remapped from `ground_projection/obslist_out`.
- Publishes: `~obslist_tracked` (ObstacleProjectedDetectionList) with smoothed locations and stable IDs.

Parameters
- `~alpha` (float): exponential smoothing factor (0..1). Default 0.5.
- `~association_radius` (m): max distance to associate detections to existing tracks. Default 0.15.
- `~max_age` (s): drop tracks older than this without updates. Default 1.0.
- `~min_hits` (int): require N hits before publishing a track. Default 1.

Launch
```
roslaunch obstacle_tracking obstacle_tracking.launch veh:=$VEHICLE_NAME
```

To route pure_pursuit to use tracked obstacles, in `lfv_start.launch` set:
```
/lane_following/obstacle_tracking:=true
```
This will start the tracker and remap pure_pursuit obs input to `obstacle_tracking/obslist_tracked`.
