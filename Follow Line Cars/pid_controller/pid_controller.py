# =============================================================================
#  Follow Line - PID controller
# =============================================================================



# The car has a single front camera and must follow the red line painted in
# the middle of the circuit as fast as possible. Every camera frame goes
# through the same steps:
#
#   1. Perception: find the red line in the image and measure how far it is
#      from the image centre, at two distances in front of the car:
#        - a FAR point (about 4-13 m ahead) that sees corners coming;
#        - a NEAR point (about 0.5-1.5 m ahead) that keeps the car on the line.
#   2. Error: blend both measurements into a single error e in [-1, 1].
#   3. Steering (the PID controller): angular speed
#      w = KP * e + KD * de + KI * I, where de is the change of the error
#      from one frame to the next and I is the accumulated error.
#   4. Speed: drive at V_MAX on straights and brake towards V_MIN when the far
#      point shows that a corner is coming.
#   5. Send (v, w) to the car and draw a debug image in the GUI.
#
# What the derivative adds over a P controller:
#   - damping: when the car is already moving back towards the line, the
#     error shrinks, de has the opposite sign and the D term reduces the
#     turn before the car overshoots, so it does not weave;
#   - anticipation: when a corner begins, the error starts growing and the
#     D term reacts to that growth before the error itself is large;
#   - tolerance to delay between the image and the command.
# That extra stability is what should allow higher speeds than with P alone.
#
# What the integral adds over a PD controller: in a corner of constant
# curvature the car needs a constant turn. P and PD can only produce it with
# a non-zero error (the car rides on the outside of the line), and the
# derivative is zero because the error does not change. The integral
# accumulates that persistent error and adds the missing turn itself, so
# the error can return to zero. It is leaky and clamped (see I_DECAY and
# I_MAX) because this circuit links corners in opposite directions, and an
# integral that never forgets would keep turning towards the previous
# corner ("integral windup").
#
# Conventions used everywhere in this file:
#   - e > 0 means the line is to the LEFT of the image centre. In ROS a
#     positive angular speed turns the car to the left (counter-clockwise),
#     so w = KP * e always steers towards the line.
#   - Speeds are in m/s, angular speeds in rad/s.
#   - "Frame" means one new camera image. The camera runs at 20 Hz of
#     simulated time, so one frame is 50 ms of simulated time.
#
# Camera geometry (from the F1 robot description used by the simulator):
# 640x480 image, 90 degree horizontal field of view (focal length 320 px),
# mounted about 10 cm above the ground, 4 cm to the left of the car's centre
# line, looking perfectly horizontal. The horizon is therefore at image row
# 240, and a point of the ground at distance d appears at row 240 + 33 / d.
# This is why the useful look-ahead (2 to 15 m) is packed into the first
# ~15 rows under the horizon, while the big red triangle at the bottom of the
# image is only a few tens of centimetres in front of the car.
# =============================================================================



import time

import cv2
import numpy as np

import HAL      # access to the robot: camera image, linear and angular speed
import WebGUI   # debug image shown in the web interface



# TUNING PARAMETERS
# =============================================================================

# Proportional gain. The error is normalised to [-1, 1], so a line at the
# very edge of the image (e = 1) produces KP rad/s of turn.
KP = 4.0             # proportional gain [rad/s per unit of normalised error]

# Derivative gain. The derivative is computed per camera frame (the change of
# the error between two consecutive frames), so KD multiplies that change.
# Too low behaves like a P controller (overshoot at corner exits); too high
# makes the steering nervous, because it amplifies the pixel noise of the
# measured line position.
KD = 6.0             # derivative gain   [rad/s per unit of error change per frame]

# Low-pass filter on the derivative: d = D_ALPHA*(new change) + (1-D_ALPHA)*d.
# The line position is measured in whole pixels, so the raw change between
# frames is noisy. 1 means no filtering; smaller values smooth more but react
# later.
D_ALPHA = 0.7        # low-pass on the derivative (1 = no filter)

# Integral term, computed per frame as I = clip(I_DECAY * I + e, -I_MAX, I_MAX).
#  - KI: weight of the accumulated error in the steering.
#  - I_DECAY < 1 makes the integral "leaky": every frame it keeps 95 % of its
#    value, so old errors fade with a memory of about 1 / (1 - 0.95) = 20
#    frames (1 s of simulated time). With a constant error e it settles at
#    e / (1 - I_DECAY) = 20 * e.
#  - I_MAX limits the accumulated error, so the integral can never add more
#    than KI * I_MAX = 1 rad/s of turn.
# Both mechanisms are the anti-windup: a too long memory (I_DECAY close to 1)
# or a too large KI makes the car overshoot at corner exits and cut the next
# corner of a chicane.
KI = 0.1             # integral gain     [rad/s per unit of accumulated error]
I_DECAY = 0.95       # leaky integrator: old error is forgotten (anti wind-up)
I_MAX = 10.0         # clamp of the accumulated error            (anti wind-up)

# Speed policy. The car runs at V_MAX while the far point is centred and slows
# down linearly towards V_MIN as the far error grows, reaching V_MIN when
# |e_far| = E_BRAKE. Because e is approximately the tangent of the angle to
# the line point, E_BRAKE = 0.5 means "full braking when the line far ahead is
# about 27 degrees off-centre". Braking is immediate, but the speed can only
# increase by ACCEL per frame, so the car does not reach a corner too fast
# right after leaving the previous one.
V_MAX = 80.0         # speed on straights [m/s]
V_MIN = 12.0         # speed in the tightest corners [m/s]
E_BRAKE = 0.5        # far-error at which we are already at V_MIN
ACCEL = 0.8          # max speed increase per frame [m/s] (braking is instant)

# How much each point contributes to the error. Most of the weight is on the
# far point, which sees corners coming; the small near contribution keeps the
# car centred. It is the same split as in the tuned P and PD controllers, so
# that each new term is the only difference from the previous controller.
FAR_WEIGHT = 0.85    # error = FAR_WEIGHT * e_far + (1 - FAR_WEIGHT) * e_near

# Turning speed used to find the line again if it disappears from the image.
W_LOST = 1.5         # turn rate used to search for the line when it is lost



# PERCEPTION
# =============================================================================

# The far band starts at the first image row that contains red ("top"), i.e.
# the farthest visible part of the line. Anchoring both bands to that row,
# instead of using fixed rows, keeps the measurement valid if the camera
# pitches slightly and in corners where the line leaves the image sideways.
HORIZON_MARGIN = 2   # rows under the horizon ignored (too far / aliased)
FAR_ROWS = 6         # far band: first rows with red under the horizon
NEAR_START = 20      # near band starts this many rows under the top of the line
NEAR_ROWS = 40       # height of the near band [rows]
MIN_PIXELS = 4       # less red than this in a band = band not usable

# Red in HSV. OpenCV stores the hue in [0, 180) and red sits at both ends of
# that range, so two intervals are needed. The saturation and value limits
# reject the grey asphalt, the white markings and dark shadows.
RED_LO1, RED_HI1 = (0, 100, 70), (10, 255, 255)      # red wraps around H = 0
RED_LO2, RED_HI2 = (165, 100, 70), (180, 255, 255)


def red_mask(roi):
    """Binary image (255 = red pixel) of the given part of the camera image."""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, RED_LO1, RED_HI1) | cv2.inRange(hsv, RED_LO2, RED_HI2)


def band_centroid(mask, r0, r1):
    """x centroid of the red pixels between rows r0 and r1 (None if too few).

    Image moments give the centroid directly (m10 / m00) and, unlike contour
    based methods, they never fail on an empty or fragmented mask.
    """
    m = cv2.moments(mask[r0:r1], binaryImage=True)
    if m["m00"] < MIN_PIXELS:
        return None
    return m["m10"] / m["m00"]


def detect_line(img):
    """Return (e_far, e_near, mask, rows) with errors normalised to [-1, 1].

    e > 0 means the line is to the LEFT of the image centre, so a positive
    angular speed (counter-clockwise in ROS) steers towards it. Because the
    focal length is 320 px, e is approximately the tangent of the angle
    between the car's heading and the measured point of the line.

    rows = (top, far_x, near_x) is only used to draw the debug image. Both
    errors are None when there is no red at all in the image.
    """
    h, w = img.shape[:2]
    top_roi = h // 2 + HORIZON_MARGIN           # camera is horizontal -> horizon at h/2
    mask = red_mask(img[top_roi:])              # only the ground half of the image
    red_rows = np.flatnonzero(mask.any(axis=1))
    if red_rows.size == 0:
        return None, None, mask, None
    top = red_rows[0]                           # farthest visible part of the line
    far_x = band_centroid(mask, top, top + FAR_ROWS)
    near_x = band_centroid(mask, top + NEAR_START, top + NEAR_START + NEAR_ROWS)
    # If only one of the bands sees the line, use it for both, so that the
    # line is only considered lost when neither band sees it.
    if near_x is None:                          # line only visible far away
        near_x = far_x
    if far_x is None:                           # far band empty, near band sees the line
        far_x = near_x
    cx = w / 2.0
    e_far = None if far_x is None else (cx - far_x) / cx
    e_near = None if near_x is None else (cx - near_x) / cx
    return e_far, e_near, mask, (top, far_x, near_x)


def debug_view(mask, rows):
    """Image shown in the GUI: the red mask (white), the far point (green),
    the near point (blue) and the vertical centre of the image (red line).

    The near point usually sits slightly to the right of the centre line even
    when the car is on the line, because the camera is mounted 4 cm to the
    left of the car's centre.
    """
    view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if rows is not None:
        top, far_x, near_x = rows
        for x, r, col in ((far_x, top + FAR_ROWS // 2, (0, 255, 0)),
                          (near_x, top + NEAR_START + NEAR_ROWS // 2, (255, 0, 0))):
            if x is not None:
                cv2.circle(view, (int(x), int(r)), 6, col, -1)
        cv2.line(view, (view.shape[1] // 2, 0), (view.shape[1] // 2, view.shape[0]), (0, 0, 255), 1)
    return view


def target_speed(e_far):
    """Fast on straights, slow when the far part of the line is off-centre.

    Linear interpolation between V_MAX (e_far = 0) and V_MIN (|e_far| >= E_BRAKE).
    The far error is used because it grows before the car reaches the corner,
    so the car brakes in advance instead of in the middle of the corner.
    """
    k = min(1.0, abs(e_far) / E_BRAKE)
    return V_MAX - (V_MAX - V_MIN) * k




# TELEMETRY
# =============================================================================

# Lap statistics measured by the controller itself, because the simulator
# interface only shows the lap time and the real-time factor. They are printed
# in the console every REPORT_EVERY frames and once more, as "LAP SUMMARY",
# after one lap. Every frame can also be saved to a CSV file. None of this is
# used by the control loop: it only measures and reports.
TELEMETRY = True
REPORT_EVERY = 40         # frames between console reports (40 frames = 2 s of sim time)
FRAME_DT = 0.05           # one camera frame = 50 ms of simulated time (camera at 20 Hz)
CAM_HEIGHT = 0.104        # camera height above the ground [m]
CAM_LEFT = 0.04           # camera mounted 4 cm to the left of the car's centre line [m]
OFFSET_ROWS = (395, 405)  # image rows where the distance to the line is measured (~0.2 m ahead)
CSV_LOG = True            # also save every frame to a CSV file inside the container
CSV_DIR = "/tmp"          # folder of the CSV file, inside the simulator container
RUN_NAME = "PID"          # prefix of the CSV file name, to tell the runs apart
LAP_LENGTH = 425.0        # distance driven in one lap [m] (red line: 429 m; the car cuts corners slightly)


class Telemetry:
    """Accumulates lap statistics and prints / saves them.

    Simulated time is counted in camera frames (frames * 50 ms), so the
    real-time factor can be computed as simulated time / wall-clock time.
    The distance travelled is the integral of the commanded speed, which the
    F1 car (holonomic model) applies directly.
    """

    def __init__(self):
        self.frames = self.lost = self.n_off = 0
        self.dist = self.sum_v = self.max_v = 0.0
        self.sum_off = self.max_off = self.sum_dw = 0.0
        self.last_w = None
        self.wall0 = None           # wall-clock time of the first frame
        self.lap_reported = False
        self.log = None             # CSV file, open while the lap is being recorded

    def open_log(self):
        """Create the CSV file of this run (the time in the name keeps runs apart)."""
        path = "%s/%s_%s.csv" % (CSV_DIR, RUN_NAME, time.strftime("%H%M%S"))
        try:
            self.log = open(path, "w")
            self.log.write("sim_time,dist,v,w,line_offset,lost\n")
            print("Logging this run to %s" % path)
        except OSError as exc:
            # The car keeps driving without the file; only the log is lost.
            print("Could not create the log file: %s" % exc)

    def line_offset(self, img, mask):
        """Lateral distance [m] between the car's centre line and the red line,
        measured on the ground just in front of the car (camera geometry).

        For a horizontal camera at height H with focal length f, a ground pixel
        in column u and row r is H*f/(r - cy) metres ahead and
        -(u - cx)*H/(r - cy) metres to the left of the camera. The line is
        measured in rows 395-405 (about 0.2 m ahead, where 1 px is about
        0.65 mm); if the line is not visible there, its lowest visible row is
        used instead. Finally the 4 cm lateral mounting offset of the camera is
        added, so that the result refers to the car's centre line. It is an
        estimate: it also contains a small part of the heading error.
        """
        h, w = img.shape[:2]
        top_roi = h // 2 + HORIZON_MARGIN          # first image row of the mask
        r0, r1 = OFFSET_ROWS
        m = cv2.moments(mask[r0 - top_roi:r1 - top_roi], binaryImage=True)
        if m["m00"] >= MIN_PIXELS:
            row = (r0 + r1 - 1) / 2.0
            u = m["m10"] / m["m00"]
        else:                                       # line not visible there: use its lowest row
            red_rows = np.flatnonzero(mask.any(axis=1))
            if red_rows.size == 0:
                return None
            row = red_rows[-1] + top_roi
            u = np.flatnonzero(mask[red_rows[-1]]).mean()
        # a pixel (u, row) on the ground is CAM_HEIGHT*f/(row-cy) m ahead and
        # -(u-cx)*CAM_HEIGHT/(row-cy) m to the left of the camera
        y_cam = -(u - (w - 1) / 2.0) * CAM_HEIGHT / (row - (h - 1) / 2.0)
        return abs(y_cam + CAM_LEFT)

    def update(self, img, mask, v, w, lost):
        """Add one frame to the statistics, with the commands just sent (v, w)."""
        if self.lap_reported:     # lap done: LAP SUMMARY stays as the last console line
            return
        if self.wall0 is None:
            self.wall0 = time.time()
            if CSV_LOG:
                self.open_log()
        self.frames += 1
        self.dist += v * FRAME_DT
        self.sum_v += v
        self.max_v = max(self.max_v, v)
        # smoothness of the steering: mean change of w between two frames
        if self.last_w is not None:
            self.sum_dw += abs(w - self.last_w)
        self.last_w = w
        off = None
        if lost:
            self.lost += 1
        else:
            off = self.line_offset(img, mask)
            if off is not None:
                self.n_off += 1
                self.sum_off += off
                self.max_off = max(self.max_off, off)
        if self.log is not None:
            self.log.write("%.2f,%.1f,%.2f,%.3f,%s,%d\n" % (
                self.frames * FRAME_DT, self.dist, v, w, "" if off is None else "%.3f" % off, lost))
        if self.frames % REPORT_EVERY == 0:
            self.report("")
            if self.log is not None:    # so that a run that goes off track still leaves its data
                self.log.flush()
        if self.dist >= LAP_LENGTH:
            self.lap_reported = True
            self.report("LAP SUMMARY ")
            if self.log is not None:
                self.log.close()
                self.log = None

    def report(self, tag):
        """Print the statistics accumulated since the start of the run."""
        sim_t = self.frames * FRAME_DT
        wall_t = max(time.time() - self.wall0, 1e-6)
        print("%s[sim %.1f s | wall %.1f s | RTF %.2f | %.0f m] speed mean %.1f max %.1f m/s | "
              "line offset mean %.1f max %.1f cm | steering mean|dw| %.2f rad/s | lost %d"
              % (tag, sim_t, wall_t, sim_t / wall_t, self.dist,
                 self.sum_v / self.frames, self.max_v,
                 100 * self.sum_off / max(self.n_off, 1), 100 * self.max_off,
                 self.sum_dw / max(self.frames - 1, 1), self.lost))




# MAIN LOOP
# =============================================================================

v = 0.0                  # current linear speed command [m/s]
last_e = 0.0             # error of the previous frame (its sign tells where the line was)
d_f = 0.0                # filtered derivative
integral = 0.0           # accumulated error
was_lost = True          # True until the line is seen (again): resets the derivative
prev_sample = None       # sub-sampled copy of the previous image, to detect new frames
telemetry = Telemetry()
seen_line = False        # becomes True the first time the line is detected
wait_frames = 0          # frames spent waiting for the line at the start

while True:
    img = HAL.getImage()

    # HAL.getImage() returns the last frame again if no new one has arrived
    # (camera = 20 Hz). Only run the controller on new frames so that the
    # loop does not steal CPU from Gazebo and the gains are per-frame.
    # Comparing a sub-sampled copy (one pixel out of 16 in each direction) is
    # enough, because consecutive camera frames always differ slightly.
    sample = img[::16, ::16]
    if prev_sample is not None and np.array_equal(sample, prev_sample):
        time.sleep(0.002)
        continue
    prev_sample = sample.copy()

    e_far, e_near, mask, rows = detect_line(img)

    if e_far is None and not seen_line:
        # camera still starting, or the car is not on the line: do not move.
        # Without this, the first empty frames would look like a lost line and
        # the car would start turning before it has seen anything.
        HAL.setV(0.0)
        HAL.setW(0.0)
        if wait_frames % REPORT_EVERY == 0:
            print("Waiting for the red line (red pixels: %d). If the car is not on the line, press Reset."
                  % cv2.countNonZero(mask))
        wait_frames += 1
        WebGUI.showImage(debug_view(mask, rows))
        continue

    if e_far is None:
        # line lost: turn towards the side where it was last seen, slowly.
        # The sign of the last error says on which side the line left the image.
        v = V_MIN * 0.5
        HAL.setV(v)
        w = W_LOST if last_e > 0 else -W_LOST
        HAL.setW(w)
        if TELEMETRY:
            telemetry.update(img, mask, v, w, lost=True)
        was_lost = True
        integral = 0.0           # do not carry old error into the recovery
        WebGUI.showImage(debug_view(mask, rows))
        continue

    if not seen_line:
        seen_line = True
        print("Line found: red pixels %d, far error %.2f, near error %.2f"
              % (cv2.countNonZero(mask), e_far, e_near))

    # Single error: mostly the far point (anticipation), plus some near point
    # (staying on the line).
    e = FAR_WEIGHT * e_far + (1.0 - FAR_WEIGHT) * e_near

    # When the line has just (re)appeared, the previous error is meaningless:
    # comparing with it would give a huge derivative (a "derivative kick").
    # The derivative memory starts again from the current error instead.
    if was_lost:             # avoid a derivative kick when the line reappears
        last_e, d_f, was_lost = e, 0.0, False


    # PID control on the steering ___________________
    
    # discrete derivative = change of the error between two consecutive frames
    # (one frame = 50 ms of simulated time), smoothed by the low-pass filter.
    d_f = D_ALPHA * (e - last_e) + (1.0 - D_ALPHA) * d_f
    # discrete integral = (leaky, clamped) sum of the error over the frames.
    # In a long corner the error has a constant sign, so the integral grows
    # and adds the turn that P and D cannot provide without an error.
    integral = max(-I_MAX, min(I_MAX, I_DECAY * integral + e))
    w = KP * e + KD * d_f + KI * integral

    # Speed: target from the far error, never increasing by more than ACCEL
    # per frame (braking, i.e. a lower target, is applied immediately).
    v = min(target_speed(e_far), v + ACCEL)
    HAL.setV(v)
    HAL.setW(w)
    if TELEMETRY:
        telemetry.update(img, mask, v, w, lost=False)

    last_e = e
    WebGUI.showImage(debug_view(mask, rows))
