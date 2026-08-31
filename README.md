# Line Follower BOT

Camera-based line following for the `vacuum_bot` differential-drive robot in ROS 2 + Gazebo. The robot watches the ground ahead of it, finds a black line, and steers to keep it centered.

## How It Works

`line_follower_node.py` subscribes to the robot's front camera and publishes steering commands, all in one callback:

1. **Get the image** — subscribes to `/model/vacuum_bot/image_raw` and converts it from ROS `Image` to an OpenCV BGR frame via `cv_bridge`.
2. **Crop to the floor ahead** — keeps only the bottom 30% of the frame, since that's the ground just in front of the robot, not the horizon.
3. **Isolate the line** — converts the crop to HSV and thresholds for dark pixels (`V` channel below 80) to build a black/white mask of the line.
4. **Find the line's center** — computes the mask's image moments; `cx = M['m10'] / M['m00']` gives the horizontal centroid of the detected line.
5. **Steer proportionally** — compares `cx` to the image's horizontal center to get an `error`, then sets:
   - `linear.x = 0.2` (constant forward speed)
   - `angular.z = -error / 120.0` (steering proportional to how far off-center the line is)
6. **Line lost** — if no dark pixels are found (`M['m00'] == 0`), it slow-rotates in place (`linear.x = 0.05`, `angular.z = 0.3`) to search for the track again.

```
Camera image ──▶ crop bottom 30% ──▶ HSV threshold ──▶ centroid ──▶ proportional steer ──▶ cmd_vel
```

## Tuning Parameters

| Parameter | Location | Default | Effect |
|---|---|---|---|
| Crop region | `image_callback` | bottom 30% of frame | How far ahead on the ground the robot "looks" |
| Black threshold | `upper_black` HSV value | `V ≤ 80` | Brightness cutoff for what counts as "line" — lower if your track/lighting picks up false positives, raise if the line isn't detected |
| Forward speed | `cmd.linear.x` | `0.2` | Constant cruising speed while on the line |
| Steering gain | `error / 120.0` | divisor `120` | Lower divisor = more aggressive steering response to being off-center |
| Search behavior | `else` branch | `linear.x=0.05`, `angular.z=0.3` | Speed/turn rate while hunting for a lost line |

## World / Track

`worlds/my_world.sdf` includes a black line track painted on the ground plane for this node to follow: one straight segment plus two curved segments, forming a simple loop-style path.

## Prerequisites

- ROS 2 (Jazzy or compatible) with `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `rqt_image_view`
- Gazebo (`gz` sim)
- Python packages: `opencv-python`, `numpy`, and `cv_bridge` (usually bundled with a ROS 2 desktop install)

```bash
pip install opencv-python numpy
```

## Build

```bash
colcon build --packages-select vacuum_bot
source install/setup.bash
```

## Run

The line follower is the node already wired into the default launch file:

```bash
ros2 launch vacuum_bot vacuum.launch.py
```

This brings up Gazebo with the line-track world, spawns the robot, bridges the camera/scan/odom topics, opens `rqt_image_view` on the robot's camera feed, and starts `line_follower_node` — the robot should immediately begin tracing the black line.

To run it standalone (e.g. sim already running):
```bash
ros2 run vacuum_bot line_follower_node.py
```

Don't run `path_planner_node.py` at the same time — both publish to `/model/vacuum_bot/cmd_vel`, and their commands will fight each other.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/model/vacuum_bot/image_raw` | `sensor_msgs/Image` | Gazebo → `line_follower_node` |
| `/model/vacuum_bot/cmd_vel` | `geometry_msgs/Twist` | `line_follower_node` → Gazebo (DiffDrive plugin) |

## Known Limitations

- Uses a single global brightness threshold — no adaptive thresholding, so changes in ambient lighting (shadows, glare) in the world can throw off detection.
- Steering is purely proportional (no integral/derivative term), so it can oscillate slightly on sharp curves rather than smoothly tracking them.
- No line-loss timeout escalation — it will search indefinitely at the same slow rotate speed rather than trying a wider search pattern if the line stays lost.
- Crop region (bottom 30%) is a fixed fraction of the frame, not tied to the camera's actual mounting height/angle — if you change `camera.xacro`'s pose, this may need adjusting.

## Restoring Missing Files

```bash
cd vacuum_bot
git status               # confirms what's missing vs. the last commit
git checkout HEAD -- .    # restores package.xml, launch/, config/, and vacuum_bot/*.py
```

Check the diff on `worlds/my_world.sdf` first — it also differs from `HEAD`, so a blanket checkout will overwrite any changes you made there too.
