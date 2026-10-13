# Lane Following

Version 0.1

Repository: https://github.com/Neobotics-Foundation-Inc/lane-following

lane_follow.py drives the NeoRacer between two lines of blue tape and keeps it in the middle of the lane. It uses the camera only.

## Files

| File | What it is |
| --- | --- |
| lane_follow.py | The program |
| bag_to_video.py | Turns a rosbag of the camera into an mp4 |
| README.md | This file |
| LICENSE | GPLv3 |

## How to run it

1. Get the program onto the car. There are two ways.

   Option A: clone the repository on the car. The car needs internet for this. Its own Wi-Fi network has no internet, so connect it by ethernet or to a Wi-Fi network that does.

   ```sh
   git clone https://github.com/Neobotics-Foundation-Inc/lane-following.git ~/lane-following
   ```

   Option B: copy the folder from your laptop. Run this from the folder that contains lane-following.

   ```sh
   scp -r lane-following racecar@neoracer:~/
   ```

2. Open a terminal inside the remote desktop session on the Jetson. The program opens a window, so it needs the desktop, not plain SSH.

3. Run it.

   ```sh
   cd ~/lane-following
   python3 lane_follow.py
   ```

4. Press Ctrl+C to stop.

The car moves as soon as the program starts if the transmitter is switched to autonomy. Keep the switch on manual, or lift the wheels, until you are ready.

## How it works

The program looks at the bottom half of the camera image and cuts it into four horizontal strips.

```
+--------------------------------------------+
|                                            |
|      top half: not used                    |
|                                            |
+--------------------------------------------+
|  strip 3   farthest, shows where the lane  |
|            is going                        |
+--------------------------------------------+
|  strip 2                                   |
+--------------------------------------------+
|  strip 1                                   |
+--------------------------------------------+
|  strip 0   nearest, shows where the car is |
+--------------------------------------------+
```

Every frame, about 60 times a second, it does this:

1. In each strip, find the blue blobs. Keep only the ones shaped like tape: a narrow stripe that runs through the strip. Blue alone is not enough, because the floor and other objects can look blue too.
2. Pick the two stripes that are a lane width apart. Those are the left and right lines.
3. Follow each line from strip to strip. A stripe only counts if it continues the line below it, in the direction that line is leaning.
4. If a line is out of view in a strip, predict where it is. First choice: next to the other line, at the lane width that strip measured earlier. Second choice: along the direction the line was heading.
5. Take the lane centre in the nearest strip. The distance from the image centre is the offset.
6. Fit a straight line through the lane centres of all the strips. The sideways shift from the nearest strip to the farthest is the heading.
7. Steering angle = KP x offset + KD x (change in offset) + K_HEADING x heading. The offset keeps the car in the middle. The heading starts the turn before the car reaches the bend.
8. If fewer than two strips see tape, keep the last steering angle.

## What you see

The window shows the camera image with the program's work drawn on it.

| Colour | Meaning |
| --- | --- |
| Gold lines | Edges of the four strips |
| Grey outline | Blue blob that was rejected (not tape-shaped) |
| White outline | Blob accepted as tape |
| Green dot | Line position that was seen |
| Orange dot | Line position that was predicted |
| Red dots and line | Lane centre in each strip, the path the car steers toward |
| Text, top left | LANE or NO LANE, then offset, heading, angle and speed |

The terminal prints one line twice a second:

```
------L--------|--------R------ : LANE    seen LR -R LR LR  offset = -0.01  heading = -0.33  angle = -0.22
```

- The bar is the nearest strip. L and R are the lines, the vertical bar is the lane centre.
- `seen` lists the strips from near to far. `LR` means both lines were seen, `-R` means only the right one.
- If it prints `No image from /camera/color`, the camera node is not publishing. Check `racecar service status`.

## Recording a run

A rosbag records the camera frames and the drive commands while the program runs. bag_to_video.py turns the camera frames into an mp4 you can watch on your laptop.

1. On the car, in a second terminal, start recording before you start the program.

   ```sh
   ros2 bag record -o lane_run_1 /camera/color /drive /odom
   ```

2. Run lane_follow.py in the first terminal and drive the lane.

3. Stop the program with Ctrl+C, then stop the recorder with Ctrl+C. The bag is a folder named lane_run_1. Use a new name for each run.

4. Turn the camera frames into a video.

   ```sh
   python3 bag_to_video.py lane_run_1
   ```

   This writes lane_run_1.mp4 next to the bag.

5. Copy the video to your laptop.

   ```sh
   scp racecar@neoracer:~/lane_run_1.mp4 <folder on your computer>
   ```

A bag is about 100 to 200 MB per minute. You can also play a bag back and run lane_follow.py against it to tune settings without driving: stop the live stack with `racecar service stop`, then `ros2 bag play lane_run_1` in one terminal and `python3 lane_follow.py` in another. With the stack stopped, nothing moves the car.

## Settings

All settings are constants at the top of lane_follow.py.

| Setting | Value | Meaning |
| --- | --- | --- |
| BLUE | (31, 40, 125) to (117, 128, 255) | HSV range for the tape |
| MIN_CONTOUR_AREA | 60 | Smallest blob to look at, in pixels |
| TAPE_WIDTH_MIN, TAPE_WIDTH_MAX | 6, 35 | Width of a tape stripe, in pixels |
| MIN_ASPECT | 2.0 | A stripe must be this many times longer than it is wide |
| MIN_LENGTH_FRAC | 0.6 | A stripe must span this fraction of the strip height |
| CROP_TOP_FRAC | 0.5 | Where the search starts, as a fraction of the image height |
| NUM_STRIPS | 4 | Number of strips |
| MAX_JUMP | 100 | Max distance between a line and where it is expected, in pixels |
| MIN_LANE_WIDTH, MAX_LANE_WIDTH | 100, 600 | Lane width limits before a strip has measured it, in pixels |
| WIDTH_TOLERANCE | 0.25 | Allowed error on the measured lane width |
| LANE_WIDTH_GUESS | 400 | Starting lane width, in pixels |
| MAX_TILT | 3.0 | Max slope of a stripe |
| MIN_STRIPS_SEEN | 2 | Strips that must see tape before the lane is trusted |
| LOST_FRAMES | 30 | Frames without a lane before the search restarts |
| HEADING_SMOOTH | 0.3 | Heading smoothing, 0 to 1 |
| SPEED | 0.1 | Constant speed, 0 to 1 (about 0.6 m/s) |
| KP, KI, KD | 2.0, 0, 0.2 | PID gains on offset |
| K_HEADING | 0.6 | Gain on heading |
| INTEGRAL_LIMIT | 0.5 | Cap on the integral term |

## Tuning

| Problem | What to change |
| --- | --- |
| The tape is not found | Retune BLUE with the HSV tuner at ~/jupyter_ws/neoracer-os/labs/utility/hsv-p_tuner.py on the car. Lower MIN_LENGTH_FRAC if the tape shows grey in the window |
| Blobs that are not tape show white | Lower TAPE_WIDTH_MAX to the real tape width plus a few pixels. Raise MIN_ASPECT to 3 |
| The car steers on clutter | Raise MIN_STRIPS_SEEN to 3 |
| The car weaves on straights | Lower KD, then KP |
| The car turns in late | Raise K_HEADING |
| The car cuts corners | Lower K_HEADING |
| Lines are dropped in sharp bends | Raise MAX_JUMP |

## Known limits

- The program only sees what the camera sees. When the inside line of a corner leaves the view, it is predicted, not seen.
- BLUE was tuned in one room. Different lights or a different floor need a retune.
- Continuous tape works best. Dashed tape needs a lower MIN_LENGTH_FRAC.
- Speed is fixed. Slowing for bends is the next step.

## License

GPLv3. Copyright Neobotics Foundation Inc.

## Changelog

- 0.1: first version. Four strips, tape-shaped stripes, prediction of hidden lines, heading from a fitted line, constant speed.
