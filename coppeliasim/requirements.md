# 📚 Exercise – Potential Field Navigation Using SLAM

## Objective

Adapt your potential-field controller to move toward bananas while avoiding
obstacles detected by a 2D laser scanner and represented in a SLAM Toolbox map.

> You may use the ground-truth banana positions provided by the simulator.
> Banana detection is not required. Replace the predefined wall positions with
> obstacles extracted from `/map`.

---

## Your task

### 1. Configure odometry and SLAM

- Begin by using the simulator's ground-truth robot pose as odometry.
- Run the 2D laser scanner and SLAM Toolbox while moving the robot.
- Verify that laser scans, `/map`, and the TF chain are updating.
- Once this setup works, repeat the experiment using wheel odometry.

### 2. Subscribe to `/map`

SLAM Toolbox combines laser measurements with odometry to build a
`nav_msgs/msg/OccupancyGrid`.

- Read its resolution, origin, width, height, and occupancy values.
- Identify occupied cells and convert their indices into positions.
- Distinguish occupied, free, and unknown cells, and define how your controller
  handles unknown areas.

### 3. Inflate obstacles in your controller

- Expand occupied regions by the robot radius plus a safety margin.
- Convert this distance into cells using the map resolution, then mark
  neighboring cells within that distance as blocked.
- The robot's center must remain outside these inflated regions.
- Update inflation whenever the map changes.

### 4. Obtain the robot pose

- Use TF to obtain the robot's position and orientation in the `map` frame.
- The banana ground-truth positions and the map share the same coordinate
  frame, so you can use the banana positions directly.

### 5. Reuse the potential field

- Bananas generate attraction, while nearby inflated obstacle boundaries
  generate repulsion.
- Process only a neighborhood around the robot.
- Avoid independently summing forces from every occupied cell, since this makes
  repulsion depend on map resolution and obstacle size.

### 6. Keep a laser-based emergency stop

- Use current scan measurements to stop when an obstacle is too close,
  including obstacles not yet represented in the map.
- Stop when all bananas have been collected.

---

## Testing

1. Begin with a completed map and low speed.
2. Test one banana in open space.
3. Test one banana near a wall.
4. Test multiple bananas.
5. Finally, test navigation while the map updates.

Repeat the tests with wheel odometry and compare map quality and navigation
behavior against the ground-truth odometry setup.

---

## Submission

Submit your code and a short demonstration video. Explain:

- your obstacle extraction;
- inflation;
- repulsive-force calculation;
- emergency stop.

Report any situations where the potential field becomes stuck.

---

## Files

- `p3_slam_toolbox.ttt` — CoppeliaSim scene for this exercise
