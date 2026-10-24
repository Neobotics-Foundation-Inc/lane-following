"""
Neobotics Foundation Inc.
GPLv3 License
lane-following/

File Name: lane_follow.py

Title: Lane Following

Author: Neobotics

Version: 0.1.0

Purpose: Drive the NeoRacer between two lines of tape and stay in the middle of the
lane using the camera only. Blue tape (the Neobotics room) or Purdue's testbed: a dashed
yellow centre line and white tape at the road's edge.

Methodology:
- Cut the floor in the camera image (below the horizon, above the car's nose) into 4
  horizontal strips, near to far.
- In each strip, find pieces of tape: narrow stripes that run through the strip.
  Tape-coloured pixels alone do not count.
- Two stripes the right distance apart are the two lane lines.
- Follow each line from strip to strip in the direction it leans. A stripe that does
  not continue the line below it is ignored.
- If a line is out of view, predict it: next to the other line at the lane width that
  strip measured earlier, or along the direction the line was heading.
- The nearest strip gives the offset from the lane centre.
- A straight line through the lane centres of all strips gives the heading, so the car
  starts turning before the bend.
- If fewer than 2 strips see tape, hold the last steering angle.

Expected Outcome:
- The NeoRacer drives forward at a fixed speed and steers to keep itself centred between
  the two lines.
- A window on the Jetson desktop shows the live camera image with:
    - the four strips (gold lines)
    - blobs that were rejected (grey) and blobs that were accepted as tape (white)
    - the seen (green) and predicted (orange) line positions
    - the lane centre path (red)
    - LANE or NO LANE and the numbers the controller is using
- If fewer than MIN_STRIPS_SEEN strips see tape, the window shows NO LANE and the car
  holds its last steering angle.
- Ctrl+C stops the program.

"""

########################################################################################
# Imports
########################################################################################

import cv2 as cv
import numpy as np

# import the racecar library.
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants <<

# HSV ranges of the tape, one per colour: a pixel in any range in TAPE is tape.
# Neobotics room: two lines of blue tape, TAPE = (BLUE,).
# Purdue testbed: a dashed yellow line down the middle of each road and white tape at its
# edges, TAPE = (YELLOW, WHITE). The red stop lines across a lane are not tape.
# YELLOW and WHITE were measured in the AutoDRIVE twin of the testbed.
# TODO: Retune with ~/jupyter_ws/neoracer-os/labs/utility/hsv-p_tuner.py on the car if needed
BLUE = ((31, 40, 125), (117, 128, 255))
YELLOW = ((20, 80, 150), (40, 255, 255))
WHITE = ((0, 0, 180), (179, 50, 255))
TAPE = (YELLOW, WHITE)

# pieces of tape of one colour this many pixels apart join into one stripe, so a dashed
# line counts as tape. 0 = no joining (the Neobotics room's tape is continuous)
# TODO: Adjust if needed
JOIN_GAP = 15

# smallest blob (pixels) to be looked at
MIN_CONTOUR_AREA = 60

# what counts as tape: a "blob" is accepted only if it is shaped like a stripe,
#   - its width is between TAPE_WIDTH_MIN and TAPE_WIDTH_MAX pixels
#   - it is at least MIN_ASPECT times longer than it is wide
#   - it spans at least MIN_LENGTH_FRAC of the strip's height, unless it touches the
#     left or right edge of the image (that is where a line leaves the view in a corner)
# TODO: Adjust if needed
TAPE_WIDTH_MIN = 6
TAPE_WIDTH_MAX = 35
MIN_ASPECT = 2.0
MIN_LENGTH_FRAC = 0.6

# CROP_TOP_FRAC - how far down the image the search starts,
# as a fraction of the height (0.5 = the bottom half). The floor starts at the
# horizon: about two thirds down for the NeoRacer's camera, which leans 12 degrees up
# (0.66 in the AutoDRIVE twin, which has the car's lens and lean)
# CROP_BOTTOM_FRAC - where it stops: 1.0 = the bottom of the image. Less if the camera
# sees the car's own nose there (0.84 in the AutoDRIVE twin)
# NUM_STRIPS - number of horizontal strips that area is cut into. Strip 0 is
# nearest the car
# TODO: Adjust if needed
CROP_TOP_FRAC = 0.66
CROP_BOTTOM_FRAC = 0.84
NUM_STRIPS = 4

# max distance (pixels) between a line and where it is expected
# TODO: Adjust if needed
MAX_JUMP = 100

# lane width limits (pixels) before a strip has measured it
# TODO: Adjust if needed
MIN_LANE_WIDTH = 100
MAX_LANE_WIDTH = 600
# allowed error on the measured lane width (fraction)
WIDTH_TOLERANCE = 0.25

# starting lane width (pixels)
# TODO: Adjust if needed
LANE_WIDTH_GUESS = 400

# max slope of a stripe (pixels sideways per pixel up)
# TODO: Adjust if needed
MAX_TILT = 3.0

# strips that must see tape before the lane is trusted
# TODO: Adjust if needed
MIN_STRIPS_SEEN = 2

# frames without a lane before the search restarts
# TODO: Adjust if needed
LOST_FRAMES = 30

# heading smoothing, 0 to 1 (smaller is smoother)
# TODO: Adjust if needed
HEADING_SMOOTH = 0.3

# constant speed, 0 to 1 (0.1 is about 0.6 m/s)
# TODO: Adjust if needed
SPEED = 0.1

# steering gains
# offset - lane centre in the nearest strip, -1 (left edge) to +1 (right edge)
# heading - sideways shift of the lane centre from the nearest strip to the farthest
# KP, KI, KD - PID gains on offset
# K_HEADING - gain on heading, so the car turns before the bend
# INTEGRAL_LIMIT - cap on the integral term
# Neobotics room: KP 2.0, K_HEADING 0.6. Purdue testbed (in the AutoDRIVE twin): KP 1.0,
# K_HEADING 0. Its bends are tight for how far ahead the camera sees (the nearest strip
# is about 0.7 m ahead of the rear wheels): more of either cuts the corners.
# KD 0 there (0.2 in the room): the offset is measured afresh each frame, and its change from
# frame to frame is mostly the detection's pixel noise, which KD multiplies by the frame rate
# (KD / dt, 12 at 0.2 and 60 Hz). At 0.2 the steering reversed on half of all commands, so the
# front wheels shook, and in the twin the car once left its lane; at 0 it reversed on 3 %
# TODO: Adjust if needed
KP = 1.0
KI = 0.0
KD = 0.0
K_HEADING = 0.0
INTEGRAL_LIMIT = 0.5

# >> Variables <<

# lane_found - "True" when enough strips see tape
# lost_count - frames in a row without a lane
# image_width - width of the last camera frame
speed = 0.0
angle = 0.0
offset = 0.0
heading = 0.0
last_offset = 0.0
integral = 0.0
lane_found = False
lost_count = 0
image_width = 640

# one entry per strip, index 0 is nearest the car
# strip_left, strip_right - line columns, seen or predicted
# left_tilt, right_tilt - slope of each line, pixels sideways per pixel up (negative = left)
# left_seen, right_seen - True where the line was in view
# strip_center - lane centre column
# strip_width - lane width the strip last measured
# width_known - "True" once measured
# memory_left, memory_right - line columns from the previous frame
strip_left = [None] * NUM_STRIPS
strip_right = [None] * NUM_STRIPS
left_tilt = [0.0] * NUM_STRIPS
right_tilt = [0.0] * NUM_STRIPS
left_seen = [False] * NUM_STRIPS
right_seen = [False] * NUM_STRIPS
strip_center = [None] * NUM_STRIPS
strip_width = [LANE_WIDTH_GUESS] * NUM_STRIPS
width_known = [False] * NUM_STRIPS
memory_left = [None] * NUM_STRIPS
memory_right = [None] * NUM_STRIPS

########################################################################################
# Functions
########################################################################################

# >> Helpers <<

"""
Identifier: segment_tilt

Usage: measures the slope of one tape stripe, so the next strip up can be searched
where the stripe points.

Parameters:
- contour: outline of one blob, as returned by rc_utils.find_contours

Outputs:
- slope of the stripe, pixels sideways per pixel up the image (negative = left)
"""
def segment_tilt(contour):
    vx, vy, _, _ = cv.fitLine(contour, cv.DIST_L2, 0, 0.01, 0.01).flatten()
    if abs(vy) < 0.05:
        return 0.0  # lying flat, its slope is meaningless
    return float(rc_utils.clamp(-vx / vy, -MAX_TILT, MAX_TILT))


"""
Identifier: tape_contours

Usage: finds the blobs of tape colour in one strip. Each colour in TAPE is masked on its
own and its pieces closer than JOIN_GAP joined, so a dashed line is one blob and two
lines of different colours never merge.

Parameters:
- strip: the strip of the camera image to search

Outputs:
- list of contours, as rc_utils.find_contours returns them
"""
def tape_contours(strip):
    hsv = cv.cvtColor(strip, cv.COLOR_BGR2HSV)
    mask = np.zeros(strip.shape[:2], np.uint8)
    for lower, upper in TAPE:
        color = cv.inRange(hsv, lower, upper)
        if JOIN_GAP > 0:
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (JOIN_GAP, JOIN_GAP))
            color = cv.morphologyEx(color, cv.MORPH_CLOSE, kernel)
        mask = cv.bitwise_or(mask, color)
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]


"""
Identifier: tape_segments

Usage: finds every blob in one strip that is shaped like tape. Outlines every blob
grey and every accepted stripe white on the strip.

Parameters:
- strip: the strip of the camera image to search (a view into the full image)
- strip_height: height of the strip in pixels
- width: width of the image in pixels

Outputs:
- list of (column, slope), one per accepted stripe
"""
def tape_segments(strip, strip_height, width):
    segments = []
    for contour in tape_contours(strip):
        if rc_utils.get_contour_area(contour) < MIN_CONTOUR_AREA:
            continue
        rc_utils.draw_contour(strip, contour, rc_utils.ColorBGR.dark_gray.value)

        # Tightest rotated box: short side = tape width, long side = tape length
        _, (side_a, side_b), _ = cv.minAreaRect(contour)
        tape_width = min(side_a, side_b)
        tape_length = max(side_a, side_b)
        x, _, w, _ = cv.boundingRect(contour)
        touches_edge = x <= 0 or x + w >= width

        if not TAPE_WIDTH_MIN <= tape_width <= TAPE_WIDTH_MAX:
            continue
        if tape_length < MIN_ASPECT * tape_width:
            continue
        if tape_length < MIN_LENGTH_FRAC * strip_height and not touches_edge:
            continue
        center = rc_utils.get_contour_center(contour)
        if center is None:
            continue

        rc_utils.draw_contour(strip, contour, rc_utils.ColorBGR.white.value)
        segments.append((center[1], segment_tilt(contour)))
    return segments


"""
Identifier: nearest_segment

Usage: picks the stripe that continues a line, which is the one nearest to where the
line is expected.

Parameters:
- segments: list of (column, slope) from tape_segments
- expected: column where the line is expected

Outputs:
- the nearest (column, slope), or None if it is more than MAX_JUMP pixels away
"""
def nearest_segment(segments, expected):
    best = None
    for segment in segments:
        if best is None or abs(segment[0] - expected) < abs(best[0] - expected):
            best = segment
    if best is None or abs(best[0] - expected) > MAX_JUMP:
        return None
    return best


"""
Identifier: lane_pair

Usage: looks for two stripes the right distance apart to be the two lane lines. Once
the strip has measured its lane width a pair must match it within WIDTH_TOLERANCE;
before that any width between MIN_LANE_WIDTH and MAX_LANE_WIDTH will do.

Parameters:
- segments: list of (column, slope) from tape_segments
- i: strip index, 0 is nearest the car
- expected_center: column where the lane centre is expected, used to choose between
  several fitting pairs

Outputs:
- (left, right) as two (column, slope) entries, or None
"""
def lane_pair(segments, i, expected_center):
    pairs = []
    for a in segments:
        for b in segments:
            gap = b[0] - a[0]
            if width_known[i]:
                fits = abs(gap - strip_width[i]) <= WIDTH_TOLERANCE * strip_width[i]
            else:
                fits = MIN_LANE_WIDTH <= gap <= MAX_LANE_WIDTH
            if fits:
                pairs.append((a, b))
    if not pairs:
        return None
    return min(pairs, key=lambda p: abs((p[0][0] + p[1][0]) / 2 - expected_center))


"""
Identifier: expected_column

Usage: works out where to look for a line in strip i: where the stripe in the strip
below points, else where the line was last frame, else the fallback.

Parameters:
- cols: this line's columns per strip, filled in so far this frame
- tilts: this line's slopes per strip
- memory: this line's columns from the previous frame
- i: strip index
- strip_height: height of a strip in pixels
- fallback: column to use when nothing else is known

Outputs:
- expected column of the line in strip i
"""
def expected_column(cols, tilts, memory, i, strip_height, fallback):
    if i > 0 and cols[i - 1] is not None:
        return cols[i - 1] + tilts[i - 1] * strip_height
    if memory[i] is not None:
        return memory[i]
    return fallback


"""
Identifier: predict_missing

Usage: fills in the strips where a line was not seen. Next to the other line a lane
width away if that line was seen there, else along this line's own slope from the
nearest strip that saw it, else left unknown.

Parameters:
- cols: this line's columns per strip, updated in place
- tilts: this line's slopes per strip, updated in place
- seen: True per strip where this line was seen
- other_cols: the other line's columns per strip
- other_seen: True per strip where the other line was seen
- sign: -1 for the left line, +1 for the right line
- strip_height: height of a strip in pixels

Outputs:
- none
- cols and tilts are filled in place
"""
def predict_missing(cols, tilts, seen, other_cols, other_seen, sign, strip_height):
    seen_at = [i for i in range(NUM_STRIPS) if seen[i]]
    for i in range(NUM_STRIPS):
        if seen[i]:
            continue
        if other_seen[i]:
            cols[i] = other_cols[i] + sign * strip_width[i]
            tilts[i] = 0.0
        elif seen_at:
            j = min(seen_at, key=lambda k: abs(k - i))
            cols[i] = cols[j] + tilts[j] * (i - j) * strip_height
            tilts[i] = tilts[j]
        else:
            cols[i] = None
            tilts[i] = 0.0


"""
Identifier: draw_strip

Usage: draws one strip's result on the image: its top edge, the line positions
(green = seen, orange = predicted) and the lane centre (red).

Parameters:
- image: the camera image to draw on
- i: strip index
- top: pixel row of the top of the strip
- bottom: pixel row of the bottom of the strip

Outputs:
- none
- draws on image
"""
def draw_strip(image, i, top, bottom):
    width = image.shape[1]
    row = (top + bottom) // 2
    cv.line(image, (0, top), (width - 1, top), (0, 215, 255), 1)
    for col, seen in ((strip_left[i], left_seen[i]), (strip_right[i], right_seen[i])):
        if col is None:
            continue
        color = (0, 255, 0) if seen else (0, 140, 255)
        cv.circle(image, (int(rc_utils.clamp(col, 0, width - 1)), row), 6, color, -1)
    if strip_center[i] is not None:
        cv.circle(image, (int(rc_utils.clamp(strip_center[i], 0, width - 1)), row), 5, (0, 0, 255), -1)


"""
Identifier: draw_overlay

Usage: draws the image centre line, the lane centre path through the strips and the
LANE / NO LANE status with the controller's numbers.

Parameters:
- image: the camera image to draw on

Outputs:
- none
- draws on image
"""
def draw_overlay(image):
    height, width = image.shape[0], image.shape[1]
    band_top = int(height * CROP_TOP_FRAC)
    band_bottom = int(height * CROP_BOTTOM_FRAC)
    strip_height = (band_bottom - band_top) // NUM_STRIPS

    cv.line(image, (width // 2, band_top), (width // 2, band_bottom), (200, 200, 200), 1)

    previous = None
    for i in range(NUM_STRIPS):
        if strip_center[i] is None:
            continue
        row = band_bottom - i * strip_height - strip_height // 2
        point = (int(rc_utils.clamp(strip_center[i], 0, width - 1)), row)
        if previous is not None:
            cv.line(image, previous, point, (0, 0, 255), 2)
        previous = point

    status = "LANE" if lane_found else "NO LANE"
    text = f"{status}   offset {offset:+.2f}   heading {heading:+.2f}   angle {angle:+.2f}   speed {speed:.2f}"
    cv.putText(image, text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


"""
Identifier: update_lines

Usage: runs the whole detection on the newest camera frame: finds the two lines in
every strip, predicts the missing ones, decides whether the lane can be trusted,
updates the heading and remembers the lines for the next frame.

Parameters:
- none

Outputs:
- the image with the detections drawn on it, or None if no frame has arrived yet
- fills strip_left, strip_right, strip_center, left_seen, right_seen, lane_found and
  heading
"""
def update_lines():
    global image_width, lane_found, lost_count, heading

    image = rc.camera.get_color_image()
    if image is None:
        for i in range(NUM_STRIPS):
            strip_center[i] = None
        lane_found = False
        return None

    # Real frame size; rc.camera.get_width()/get_height() return fixed constants
    height, width = image.shape[0], image.shape[1]
    image_width = width
    band_top = int(height * CROP_TOP_FRAC)
    band_bottom = int(height * CROP_BOTTOM_FRAC)
    strip_height = (band_bottom - band_top) // NUM_STRIPS

    for i in range(NUM_STRIPS):
        # Strip 0 at the bottom of the search area. crop() is a view, so drawing on the
        # strip draws on the full image
        bottom = band_bottom - i * strip_height
        top = bottom - strip_height
        strip = rc_utils.crop(image, (top, 0), (bottom, width))

        expected_left = expected_column(
            strip_left, left_tilt, memory_left, i, strip_height, width // 2 - strip_width[i] // 2)
        expected_right = expected_column(
            strip_right, right_tilt, memory_right, i, strip_height, width // 2 + strip_width[i] // 2)

        segments = tape_segments(strip, strip_height, width)

        # Best case: a pair the right distance apart. Else each line is the nearest
        # stripe to where it is expected
        pair = lane_pair(segments, i, (expected_left + expected_right) / 2)
        if pair is not None:
            left, right = pair
        else:
            left = nearest_segment(segments, expected_left)
            right = nearest_segment(segments, expected_right)
            # One stripe cannot be both lines; keep the one nearer its expected spot
            if left is not None and right is not None and right[0] - left[0] < MIN_LANE_WIDTH:
                if abs(left[0] - expected_left) <= abs(right[0] - expected_right):
                    right = None
                else:
                    left = None

        left_seen[i] = left is not None
        right_seen[i] = right is not None
        strip_left[i], left_tilt[i] = (left if left is not None else (None, 0.0))
        strip_right[i], right_tilt[i] = (right if right is not None else (None, 0.0))

        if left is not None and right is not None:
            strip_width[i] = right[0] - left[0]
            width_known[i] = True

    # Predict the lines that were out of view, then mark the lane centre per strip
    predict_missing(strip_left, left_tilt, left_seen, strip_right, right_seen, -1, strip_height)
    predict_missing(strip_right, right_tilt, right_seen, strip_left, left_seen, +1, strip_height)
    for i in range(NUM_STRIPS):
        if strip_left[i] is None or strip_right[i] is None:
            strip_center[i] = None
        else:
            strip_center[i] = (strip_left[i] + strip_right[i]) / 2

    seen_strips = [i for i in range(NUM_STRIPS) if left_seen[i] or right_seen[i]]
    lane_found = len(seen_strips) >= MIN_STRIPS_SEEN

    # heading from a straight line through the lane centres of the strips that saw tape.
    # With one strip, use the slope of its tape
    half_width = width / 2
    new_heading = None
    if len(seen_strips) >= 2:
        slope = np.polyfit(seen_strips, [strip_center[i] for i in seen_strips], 1)[0]
        new_heading = slope * (NUM_STRIPS - 1) / half_width
    elif len(seen_strips) == 1:
        i = seen_strips[0]
        tilts = [t for t, s in ((left_tilt[i], left_seen[i]), (right_tilt[i], right_seen[i])) if s]
        new_heading = (sum(tilts) / len(tilts)) * (NUM_STRIPS - 1) * strip_height / half_width
    if new_heading is not None:
        heading += HEADING_SMOOTH * (rc_utils.clamp(new_heading, -1.0, 1.0) - heading)

    # Remember the lines for next frame; forget them after LOST_FRAMES without a lane
    if lane_found:
        lost_count = 0
        for i in range(NUM_STRIPS):
            memory_left[i] = strip_left[i]
            memory_right[i] = strip_right[i]
    else:
        lost_count += 1
        if lost_count >= LOST_FRAMES:
            for i in range(NUM_STRIPS):
                memory_left[i] = None
                memory_right[i] = None

    for i in range(NUM_STRIPS):
        draw_strip(image, i, band_bottom - (i + 1) * strip_height, band_bottom - i * strip_height)
    return image


# >> Main <<

"""
Identifier: start

Usage: runs once when the program starts. Resets every variable, stops the car and
prints the start message.

Parameters:
- none

Outputs:
- none
"""
def start():
    global speed, angle, offset, heading, last_offset, integral, lane_found, lost_count

    speed = 0.0
    angle = 0.0
    offset = 0.0
    heading = 0.0
    last_offset = 0.0
    integral = 0.0
    lane_found = False
    lost_count = 0
    for i in range(NUM_STRIPS):
        strip_left[i] = None
        strip_right[i] = None
        strip_center[i] = None
        left_tilt[i] = 0.0
        right_tilt[i] = 0.0
        left_seen[i] = False
        right_seen[i] = False
        strip_width[i] = LANE_WIDTH_GUESS
        width_known[i] = False
        memory_left[i] = None
        memory_right[i] = None

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)

    print(
        ">> Lane Following - Two Lines of Tape\n"
        "\n"
        f"Speed is fixed at {SPEED}. Steering = PID on the lane offset in the nearest strip\n"
        f"(KP = {KP}, KI = {KI}, KD = {KD}) + {K_HEADING} x the lane heading fitted through all strips.\n"
        f"The window shows {NUM_STRIPS} strips: grey = tape-coloured blob rejected, white = tape stripe,\n"
        "green = line seen, orange = line predicted, red = lane centre. Press Ctrl+C to stop."
    )


"""
Identifier: update

Usage: runs every frame, about 60 times a second. Finds the lane, computes the
steering angle with the PID and the heading term, sends speed and angle to the car
and shows the camera window.

Parameters:
- none

Outputs:
- none
- sends one drive command to the car per frame
"""
def update():
    global speed, angle, offset, last_offset, integral

    image = update_lines()

    # Steer only on a trusted lane; otherwise keep the previous angle
    if lane_found and strip_center[0] is not None:
        half_width = image.shape[1] / 2

        # Offset of the lane centre from the image centre, -1..1, positive = lane to the right
        offset = (strip_center[0] - half_width) / half_width

        dt = rc.get_delta_time()
        integral = rc_utils.clamp(integral + offset * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        derivative = (offset - last_offset) / dt if dt > 0 else 0.0
        last_offset = offset

        # PID on offset plus feed-forward on heading (computed in update_lines)
        angle = rc_utils.clamp(
            KP * offset + KI * integral + KD * derivative + K_HEADING * heading, -1.0, 1.0
        )

    speed = SPEED
    rc.drive.set_speed_angle(speed, angle)

    if image is not None:
        draw_overlay(image)
        rc.display.show_color_image(image)


"""
Identifier: update_slow

Usage: runs twice a second. Prints one line showing the nearest strip, which strips
saw which lines, and the controller's numbers.

Parameters:
- none

Outputs:
- none
- prints to the terminal
"""
def update_slow():
    # No frame on /camera/color yet: the camera node, not this script
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image from /camera/color: is the camera node running?) " + "X" * 10)
        return

    # bar of the nearest strip (L, R, | = lane centre), which strips saw which lines,
    # and the numbers
    s = ["-"] * 32
    bucket = max(1, image_width // 32)
    for col, mark in ((strip_left[0], "L"), (strip_right[0], "R"), (strip_center[0], "|")):
        if col is not None:
            s[int(rc_utils.clamp(col // bucket, 0, 31))] = mark

    seen = " ".join(
        ("L" if left_seen[i] else "-") + ("R" if right_seen[i] else "-")
        for i in range(NUM_STRIPS)
    )
    status = "LANE   " if lane_found else "NO LANE"
    print(
        "".join(s)
        + f" : {status} seen {seen}  offset = {offset:+.2f}  heading = {heading:+.2f}  angle = {angle:+.2f}"
    )


########################################################################################
#! DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
