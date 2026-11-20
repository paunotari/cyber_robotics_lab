# =============================================================================
#  Rescue People - search for castaways with a drone and face detection
# =============================================================================



# A quadrotor leaves a safety boat, flies to the area where a group of
# castaways was last reported, sweeps that area with its downward-looking
# (ventral) camera, recognises the face of every person floating face-up,
# stores the position of each one and flies back to the boat with the list.
#
# The mission is a sequence of phases (a small state machine):
#   1. TAKEOFF  take off from the boat to the search altitude.
#   2. TRANSIT  fly to the reported position of the castaways. It is given in
#               GPS coordinates and converted to metres relative to the boat.
#   3. SEARCH   sweep the area along an Archimedean spiral centred on that
#               position. Every camera image is searched for faces.
#   4. CONFIRM  when a face that is not on the list yet appears, leave the
#               spiral, fly right over it and look again with a finer search.
#               If the face is still there, store its position; if not, it was
#               a false detection. Then go back to the spiral where it was left.
#   5. RETURN   when all the castaways have been found, or the spiral has
#               covered the whole area, fly back to the boat...
#   6. LAND     ...land on it and print the list of positions.
#
# Conventions used everywhere in this file:
#   - Positions are in metres in the local frame of HAL.get_position() and
#     HAL.set_cmd_pos(): origin at the boat, x towards the East, y towards the
#     North, z up. The reported GPS position of the castaways converts to
#     about 40 m East and 30 m South of the boat, i.e. x ~ 40, y ~ -30.
#   - Angles are in radians, except the image rotation steps (degrees).
#   - The drone keeps the yaw it has after takeoff for the whole mission
#     ("heads-free" flight): set_cmd_pos moves it in any direction without
#     turning, so the camera footprint always has the same orientation.
#
# Camera geometry (ventral camera of the simulated quadrotor): 320x240 image,
# horizontal field of view 1.03 rad (59 degrees), looking straight down. The
# top of the image points to the front of the drone and the right of the
# image to its right. With a focal length of f = 160 / tan(0.515) ~ 283 px,
# a camera at height h sees a rectangle of water of 1.13 h x 0.85 h metres,
# and 1 m on the water measures f / h pixels in the image.
#
# What a castaway looks like from above: a picture of a person lying on the
# water, 1.5 m wide and 3 m long, with a real face photograph at the head.
# The face measures about 0.45 m x 0.9 m on the water: the photograph is
# stretched about 1.4 times along the body, so faces look longer than real
# ones. People float with any orientation, and the face detector (a Haar
# cascade trained on upright frontal faces) only finds faces that are within
# about 15 degrees of upright. The image is therefore rotated in steps, and
# after each rotation it is also compressed vertically to undo the stretch.
# =============================================================================
import csv
import glob
import math
import os
import time

import cv2
import numpy as np

import HAL      # access to the drone: position, yaw, cameras, commands
import WebGUI   # images shown in the web interface



# TUNING PARAMETERS
# =============================================================================

# Reported positions, as (degrees, minutes, seconds). Latitudes are North and
# longitudes West. Only the difference between both points is used: it gives
# where the castaways are relative to the boat, in metres.
BOAT_LAT, BOAT_LON = (40, 16, 48.2), (3, 49, 3.5)
SURVIVORS_LAT, SURVIVORS_LON = (40, 16, 47.23), (3, 49, 1.78)
N_SURVIVORS = 6              # castaways to find before going back

# Flight. The search altitude is a trade-off: higher means a larger camera
# footprint (fewer spiral turns), lower means larger faces in the image.
# At 4 m the camera sees 4.5 m x 3.4 m of water and a face measures about
# 30 x 65 pixels in the camera image, above the detector's minimum size.
SEARCH_ALTITUDE = 4.0        # flight altitude for the whole mission [m]
ARRIVE_TOL = 0.8             # distance at which a waypoint counts as reached [m]
HOVER_TOL = 0.35             # distance to hover over a face before checking it [m]

# Archimedean spiral r = b * theta, centred on the reported position. One
# turn moves the path outwards by SPIRAL_SPACING metres (b = spacing / 2 pi).
# The spacing is smaller than the narrowest side of the camera footprint
# (3.4 m at 4 m), so consecutive turns overlap and no strip of water is
# left unseen. The maximum radius covers the area around the reported point
# with a margin, since the castaways drift around it.
SPIRAL_SPACING = 2.5         # distance between consecutive turns [m]
SPIRAL_MAX_RADIUS = 22.0     # radius at which the search ends [m]
SPIRAL_STEP = 0.5            # distance between spiral waypoints [m]
LOOKAHEAD = 2.0              # how far ahead of the drone the target point runs [m]

# Face detection. Each rotated image is searched as it is (1.0) and
# compressed vertically (FACE_SQUASH), so that faces are found whether or not
# the picture is stretched on the water as expected.
UPSCALE = 1.25               # image enlargement before detection
ROT_STEP_SEARCH = 30         # rotation step while searching [deg] (12 rotations)
ROT_STEP_CONFIRM = 15        # finer step when checking a candidate [deg]
FACE_SQUASH = 0.7            # vertical compression that undoes the 1.4x stretch
SQUASHES = (FACE_SQUASH, 1.0)
MIN_FACE = 28                # smallest face searched, in the enlarged image [px]
MIN_NEIGHBORS = 4            # Haar cascade: higher = fewer false detections
MERGE_PX = 20                # detections closer than this are the same face [px]

# Bookkeeping of castaways. The closest two castaways are about 4 m apart,
# so two positions closer than SAME_SURVIVOR_RADIUS are the same person.
SAME_SURVIVOR_RADIUS = 2.0   # [m]
CANDIDATE_RADIUS = 1.5       # detections closer than this feed the same candidate [m]
CANDIDATE_HITS = 2           # images in which a new face must appear to stop for it
CANDIDATE_MEMORY = 8         # a candidate not seen in this many images is dropped
REJECT_RADIUS = 1.5          # no new candidates this close to a rejected one [m]
CONFIRM_MISSES = 12          # consecutive images without the face before rejecting it
CONFIRM_TIMEOUT = 40.0       # give up a candidate after this long [s of wall time]
CONFIRM_SAMPLES = 3          # centred detections averaged into the final position
CONFIRM_CENTRE_PX = 30       # "centred" = this close to the image centre [px]
MAX_RETARGETS = 3            # corrections of the hover point before accepting

# Camera model.
CAM_HFOV = 1.02974           # horizontal field of view [rad]
CAM_BELOW_BODY = 0.05        # camera mounted 5 cm below the drone's centre [m]
WATER_LEVEL = 0.0            # height of the water surface in the local frame [m]

# Log of the mission (one CSV row per control cycle), inside the container.
CSV_LOG = True
CSV_DIR = "/tmp"



# GEOGRAPHY: GPS <-> LOCAL METRES
# =============================================================================
# Over a few tens of metres the Earth can be treated as flat: one degree of
# latitude is R * pi / 180 metres, and one degree of longitude the same times
# cos(latitude). This is enough to place the search area and to report the
# positions found back in GPS coordinates.

EARTH_RADIUS = 6371000.0


def dms_to_deg(dms):
    """(degrees, minutes, seconds) -> decimal degrees."""
    d, m, s = dms
    return d + m / 60.0 + s / 3600.0


BOAT_LAT_DEG = dms_to_deg(BOAT_LAT)
BOAT_LON_DEG = -dms_to_deg(BOAT_LON)        # West is negative


def gps_to_local(lat_dms, lon_dms):
    """GPS position (North latitude, West longitude) -> (x East, y North) in
    metres relative to the boat."""
    lat = dms_to_deg(lat_dms)
    lon = -dms_to_deg(lon_dms)
    y = math.radians(lat - BOAT_LAT_DEG) * EARTH_RADIUS
    x = math.radians(lon - BOAT_LON_DEG) * EARTH_RADIUS * math.cos(math.radians(BOAT_LAT_DEG))
    return x, y


def local_to_gps_text(x, y):
    """(x East, y North) relative to the boat -> readable GPS text."""
    lat = BOAT_LAT_DEG + math.degrees(y / EARTH_RADIUS)
    lon = BOAT_LON_DEG + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(BOAT_LAT_DEG))))

    def fmt(value, pos, neg):
        hemi = pos if value >= 0 else neg
        value = abs(value)
        d = int(value)
        m = int((value - d) * 60)
        s = (value - d - m / 60.0) * 3600
        return "%d°%02d'%05.2f\"%s" % (d, m, s, hemi)
    return "%s %s" % (fmt(lat, "N", "S"), fmt(lon, "E", "W"))



# SEARCH PATH: ARCHIMEDEAN SPIRAL
# =============================================================================

def spiral_waypoints(cx, cy, spacing, max_radius, step):
    """Points every `step` metres along r = b * theta around (cx, cy).

    The angle increment is chosen so that consecutive points are `step`
    metres apart along the curve: the length of a small piece of the spiral
    is sqrt(r^2 + b^2) * d_theta, so d_theta = step / sqrt(r^2 + b^2).
    The points are then x = cx + r cos(theta), y = cy + r sin(theta).
    """
    b = spacing / (2.0 * math.pi)
    theta, points = 0.0, []
    while True:
        r = b * theta
        points.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
        if r >= max_radius:
            break
        theta += step / math.sqrt(r * r + b * b)
    return np.array(points)



# PERCEPTION: FACE DETECTION
# =============================================================================

def find_cascade_path():
    """Location of the frontal-face Haar cascade inside the container.

    The exercise provides it under /resources; OpenCV also ships a copy.
    A few known places are tried first, then a limited search.
    """
    candidates = ["/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"]
    data = getattr(cv2, "data", None)
    if data is not None:
        candidates.append(os.path.join(data.haarcascades, "haarcascade_frontalface_default.xml"))
    candidates += ["/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
                   "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
                   "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"]
    for path in candidates:
        if os.path.exists(path):
            return path
    for root in ("/resources", "/opt", "/usr/share", "/usr/local", "/RoboticsAcademy"):
        found = glob.glob(os.path.join(root, "**", "haarcascade_frontalface_default.xml"), recursive=True)
        if found:
            return found[0]
    return None


def detect_faces(img, rot_step, squashes):
    """Faces in a BGR image, as a list of dicts {u, v, votes, size}.

    (u, v) is the centre of the face in pixels of the original image, `votes`
    how many rotated versions of the image found it (a confidence measure)
    and `size` its approximate width in original pixels.

    For every rotation angle the image is rotated on a canvas large enough to
    keep all of it, optionally compressed vertically (FACE_SQUASH, to undo the
    stretch of the face pictures), and passed to the Haar cascade. The centre
    of each detected box is then mapped back to the original image with the
    inverse of the same transformation.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if UPSCALE != 1.0:
        gray = cv2.resize(gray, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_LINEAR)
    h, w = gray.shape
    centre = (w / 2.0, h / 2.0)
    points = []
    for angle in range(0, 360, rot_step):
        rot_m = cv2.getRotationMatrix2D(centre, angle, 1.0)
        cos_a, sin_a = abs(rot_m[0, 0]), abs(rot_m[0, 1])
        new_w = int(h * sin_a + w * cos_a)
        new_h = int(h * cos_a + w * sin_a)
        rot_m[0, 2] += new_w / 2.0 - centre[0]      # keep the rotated image centred
        rot_m[1, 2] += new_h / 2.0 - centre[1]
        rotated = cv2.warpAffine(gray, rot_m, (new_w, new_h))
        inverse = cv2.invertAffineTransform(rot_m)
        for squash in squashes:
            if squash == 1.0:
                view = rotated
            else:
                view = cv2.resize(rotated, (new_w, max(1, int(new_h * squash))))
            faces = cascade.detectMultiScale(view, scaleFactor=1.1, minNeighbors=MIN_NEIGHBORS,
                                             minSize=(MIN_FACE, MIN_FACE))
            for (x, y, fw, fh) in faces:
                # box centre in the rotated canvas (undo the squash first)
                rx, ry = x + fw / 2.0, (y + fh / 2.0) / squash
                u = inverse[0, 0] * rx + inverse[0, 1] * ry + inverse[0, 2]
                v = inverse[1, 0] * rx + inverse[1, 1] * ry + inverse[1, 2]
                points.append((u / UPSCALE, v / UPSCALE, fw / UPSCALE))
    return merge_detections(points)


def merge_detections(points):
    """The same face is usually found in several rotations: group the points
    closer than MERGE_PX and return one averaged detection per group."""
    groups = []
    for u, v, size in points:
        for g in groups:
            if math.hypot(u - g["u"], v - g["v"]) < MERGE_PX:
                n = g["votes"]
                g["u"] = (g["u"] * n + u) / (n + 1)
                g["v"] = (g["v"] * n + v) / (n + 1)
                g["size"] = max(g["size"], size)
                g["votes"] = n + 1
                break
        else:
            groups.append({"u": u, "v": v, "votes": 1, "size": size})
    return groups


def pixel_to_world(u, v, img_shape, pos, yaw):
    """Pixel of the ventral camera -> (x, y) of that point on the water.

    The camera looks straight down from height h above the water. A point
    du pixels right and dv pixels below the image centre is
    (-dv * h / f) metres in front of the drone and (du * h / f) metres to its
    right. Both are then rotated by the yaw of the drone into the local frame:
    forward = (cos yaw, sin yaw) and right = (sin yaw, -cos yaw).
    """
    img_h, img_w = img_shape[:2]
    f = (img_w / 2.0) / math.tan(CAM_HFOV / 2.0)
    height = max(0.5, pos[2] - CAM_BELOW_BODY - WATER_LEVEL)
    du = u - (img_w - 1) / 2.0
    dv = v - (img_h - 1) / 2.0
    forward = -dv * height / f
    right = du * height / f
    x = pos[0] + forward * math.cos(yaw) + right * math.sin(yaw)
    y = pos[1] + forward * math.sin(yaw) - right * math.cos(yaw)
    return x, y


def offset_from_centre(det, img_shape):
    """Distance in pixels between a detection and the image centre."""
    img_h, img_w = img_shape[:2]
    return math.hypot(det["u"] - (img_w - 1) / 2.0, det["v"] - (img_h - 1) / 2.0)



# DISPLAY: MAP OF THE MISSION AND CAMERA VIEW
# =============================================================================

class MissionMap:
    """Top view of the mission: boat, planned spiral, path flown, castaways
    found and rejected detections. North is up, East to the right."""

    SCALE = 6.0                                  # pixels per metre

    def __init__(self, home, spiral):
        xs = [home[0]] + list(spiral[:, 0])
        ys = [home[1]] + list(spiral[:, 1])
        self.xmin, self.xmax = min(xs) - 5.0, max(xs) + 5.0
        self.ymin, self.ymax = min(ys) - 5.0, max(ys) + 5.0
        width = int((self.xmax - self.xmin) * self.SCALE)
        height = int((self.ymax - self.ymin) * self.SCALE)
        self.base = np.full((height, width, 3), (110, 70, 30), np.uint8)   # sea
        pts = np.array([self.px(x, y) for x, y in spiral], np.int32)
        cv2.polylines(self.base, [pts], False, (170, 140, 100), 1)
        bx, by = self.px(home[0], home[1])
        cv2.rectangle(self.base, (bx - 6, by - 4), (bx + 6, by + 4), (255, 255, 255), -1)
        cv2.putText(self.base, "boat", (bx + 8, by + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        self.trail = []

    def px(self, x, y):
        return (int((x - self.xmin) * self.SCALE), int((self.ymax - y) * self.SCALE))

    def render(self, pos, state, survivors, rejected, target):
        if not self.trail or math.hypot(pos[0] - self.trail[-1][0], pos[1] - self.trail[-1][1]) > 0.3:
            self.trail.append((pos[0], pos[1]))
        img = self.base.copy()
        if len(self.trail) > 1:
            pts = np.array([self.px(x, y) for x, y in self.trail], np.int32)
            cv2.polylines(img, [pts], False, (0, 220, 255), 1)
        for x, y in rejected:
            cx, cy = self.px(x, y)
            cv2.line(img, (cx - 4, cy - 4), (cx + 4, cy + 4), (150, 150, 150), 1)
            cv2.line(img, (cx - 4, cy + 4), (cx + 4, cy - 4), (150, 150, 150), 1)
        for i, s in enumerate(survivors):
            cx, cy = self.px(s["x"], s["y"])
            cv2.circle(img, (cx, cy), 6, (0, 255, 0), -1)
            cv2.putText(img, str(i + 1), (cx + 7, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        if target is not None:
            cv2.circle(img, self.px(target[0], target[1]), 5, (0, 255, 255), 1)
        cv2.circle(img, self.px(pos[0], pos[1]), 4, (0, 0, 255), -1)
        cv2.putText(img, "%s  found %d/%d" % (state, len(survivors), N_SURVIVORS), (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img


def camera_view(img, detections, state):
    """Ventral image with the detected faces and the image centre marked."""
    view = img.copy()
    h, w = view.shape[:2]
    cv2.line(view, (w // 2 - 8, h // 2), (w // 2 + 8, h // 2), (0, 255, 255), 1)
    cv2.line(view, (w // 2, h // 2 - 8), (w // 2, h // 2 + 8), (0, 255, 255), 1)
    for d in detections:
        r = max(8, int(d["size"] / 2))
        cv2.circle(view, (int(d["u"]), int(d["v"])), r, (0, 255, 0), 2)
        cv2.putText(view, str(d["votes"]), (int(d["u"]) + r, int(d["v"])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(view, state, (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return view



# MISSION
# =============================================================================

def distance_2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def near_any(point, places, radius, key=None):
    """True if `point` is within `radius` of any of `places`."""
    for p in places:
        q = (p["x"], p["y"]) if key else p
        if distance_2d(point, q) < radius:
            return True
    return False


class MissionLog:
    """One CSV row per control cycle: time, phase, position, castaways found
    and an optional event (candidate, confirmation, rejection...)."""

    def __init__(self):
        self.file = None
        self.t0 = time.time()
        if CSV_LOG:
            path = "%s/RESCUE_%s.csv" % (CSV_DIR, time.strftime("%H%M%S"))
            try:
                self.file = open(path, "w", newline="")
                self.writer = csv.writer(self.file)
                self.writer.writerow(["time", "state", "x", "y", "z", "yaw", "found", "event"])
                print("Logging this mission to %s" % path)
            except OSError as exc:
                print("Could not create the log file: %s" % exc)
        self.last_flush = time.time()

    def row(self, state, pos, yaw, found, event=""):
        if self.file is None:
            return
        self.writer.writerow(["%.2f" % (time.time() - self.t0), state, "%.2f" % pos[0], "%.2f" % pos[1],
                              "%.2f" % pos[2], "%.3f" % yaw, found, event])
        if event or time.time() - self.last_flush > 2.0:
            self.file.flush()
            self.last_flush = time.time()


def say(log, state, pos, yaw, found, text):
    """Print an event in the console and store it in the log."""
    print(text)
    log.row(state, pos, yaw, found, text)


def run_mission():
    global cascade

    # Face detector
    cascade_path = find_cascade_path()
    cascade = cv2.CascadeClassifier(cascade_path) if cascade_path else None
    if cascade is None or cascade.empty():
        print("Face detector not found: haarcascade_frontalface_default.xml")
        while True:
            time.sleep(1.0)
    print("Face detector: %s" % cascade_path)

    # Search area: reported position of the castaways, relative to the boat.
    area_x, area_y = gps_to_local(SURVIVORS_LAT, SURVIVORS_LON)
    print("Reported castaway position: x = %.1f m, y = %.1f m from the boat" % (area_x, area_y))

    # TAKEOFF (blocking: returns at the search altitude)
    home = HAL.get_position().copy()
    print("Taking off from x = %.1f, y = %.1f" % (home[0], home[1]))
    HAL.takeoff(SEARCH_ALTITUDE)
    yaw0 = HAL.get_yaw()

    spiral = spiral_waypoints(area_x, area_y, SPIRAL_SPACING, SPIRAL_MAX_RADIUS, SPIRAL_STEP)
    print("Spiral: %d waypoints, %.0f m long" % (len(spiral), SPIRAL_STEP * (len(spiral) - 1)))
    mission_map = MissionMap(home, spiral)
    log = MissionLog()

    survivors = []        # confirmed castaways: {"x", "y"}
    rejected = []         # positions of detections that were not confirmed
    candidates = []       # faces seen while searching, not yet checked
    spiral_idx = 0        # current target point on the spiral
    frame_no = 0
    state = "TRANSIT"
    target = (area_x, area_y)
    confirm = None        # data of the candidate being checked
    detections = []

    while True:
        pos = HAL.get_position()
        yaw = HAL.get_yaw()
        img = HAL.get_ventral_image()
        frame_no += 1
        detections = []
        n = len(survivors)

        if state == "TRANSIT":
            # Straight to the centre of the search area (the start of the spiral).
            target = (area_x, area_y)
            if distance_2d(pos, target) < ARRIVE_TOL:
                state = "SEARCH"
                say(log, state, pos, yaw, n, "Search area reached: starting the spiral")

        elif state == "SEARCH":
            # Move the target point along the spiral while the drone is close to
            # it, so that it always stays about LOOKAHEAD metres ahead.
            while spiral_idx < len(spiral) - 1 and distance_2d(pos, spiral[spiral_idx]) < LOOKAHEAD:
                spiral_idx += 1
            target = tuple(spiral[spiral_idx])

            detections = detect_faces(img, ROT_STEP_SEARCH, SQUASHES)
            for d in detections:
                p = pixel_to_world(d["u"], d["v"], img.shape, pos, yaw)
                if near_any(p, survivors, SAME_SURVIVOR_RADIUS, key=True) or near_any(p, rejected, REJECT_RADIUS):
                    continue                    # already on the list, or a known false detection
                for c in candidates:
                    if distance_2d(p, (c["x"], c["y"])) < CANDIDATE_RADIUS:
                        k = c["hits"]
                        c["x"], c["y"] = (c["x"] * k + p[0]) / (k + 1), (c["y"] * k + p[1]) / (k + 1)
                        c["hits"], c["seen"] = k + 1, frame_no
                        break
                else:
                    candidates.append({"x": p[0], "y": p[1], "hits": 1, "seen": frame_no})
            candidates = [c for c in candidates if frame_no - c["seen"] <= CANDIDATE_MEMORY]

            ready = [c for c in candidates if c["hits"] >= CANDIDATE_HITS]
            if ready:
                c = max(ready, key=lambda c: c["hits"])
                candidates.remove(c)
                confirm = {"target": (c["x"], c["y"]), "samples": [], "misses": 0,
                           "retargets": 0, "last_offset": None, "start": time.time()}
                state = "CONFIRM"
                say(log, state, pos, yaw, n, "Face seen at x = %.1f, y = %.1f: checking it" % (c["x"], c["y"]))
            elif len(survivors) >= N_SURVIVORS:
                state = "RETURN"
                say(log, state, pos, yaw, n, "All %d castaways found: returning to the boat" % N_SURVIVORS)
            elif spiral_idx == len(spiral) - 1 and distance_2d(pos, spiral[-1]) < ARRIVE_TOL:
                state = "RETURN"
                say(log, state, pos, yaw, n, "Spiral completed with %d castaways found: returning" % n)

        elif state == "CONFIRM":
            # Hover right above the candidate and look again, more carefully.
            target = confirm["target"]
            if time.time() - confirm["start"] > CONFIRM_TIMEOUT:
                rejected.append(target)
                state = "SEARCH"
                say(log, state, pos, yaw, n, "Could not check x = %.1f, y = %.1f in time: skipped" % target)
            elif distance_2d(pos, target) < HOVER_TOL:
                detections = detect_faces(img, ROT_STEP_CONFIRM, SQUASHES)
                best, best_dist = None, None
                for d in detections:
                    p = pixel_to_world(d["u"], d["v"], img.shape, pos, yaw)
                    dist = distance_2d(p, target)
                    if dist < SAME_SURVIVOR_RADIUS and (best is None or dist < best_dist):
                        best, best_dist, best_p = d, dist, p
                if best is None:
                    confirm["misses"] += 1
                    if confirm["misses"] >= CONFIRM_MISSES:
                        rejected.append(target)
                        state = "SEARCH"
                        say(log, state, pos, yaw, n, "No face at x = %.1f, y = %.1f: false detection" % target)
                else:
                    confirm["misses"] = 0
                    off = offset_from_centre(best, img.shape)
                    if off <= CONFIRM_CENTRE_PX or confirm["retargets"] >= MAX_RETARGETS:
                        confirm["samples"].append(best_p)
                        if len(confirm["samples"]) >= CONFIRM_SAMPLES:
                            sx = float(np.median([s[0] for s in confirm["samples"]]))
                            sy = float(np.median([s[1] for s in confirm["samples"]]))
                            if near_any((sx, sy), survivors, SAME_SURVIVOR_RADIUS, key=True):
                                text = "Face at x = %.1f, y = %.1f was already on the list" % (sx, sy)
                            else:
                                survivors.append({"x": sx, "y": sy})
                                text = "Castaway %d confirmed at x = %.1f, y = %.1f" % (len(survivors), sx, sy)
                            state = "SEARCH"
                            say(log, state, pos, yaw, len(survivors), text)
                    else:
                        # The face is not under the drone yet: hover over its new
                        # estimate. If each correction moves the face further
                        # from the centre, the camera axes are not as assumed.
                        if confirm["last_offset"] is not None and off > confirm["last_offset"] + 10:
                            print("Warning: the face moved away from the image centre after "
                                  "the correction (%.0f -> %.0f px)" % (confirm["last_offset"], off))
                        confirm["last_offset"] = off
                        confirm["target"] = best_p
                        confirm["retargets"] += 1

        elif state == "RETURN":
            target = (home[0], home[1])
            if distance_2d(pos, target) < ARRIVE_TOL:
                state = "LAND"
                say(log, state, pos, yaw, len(survivors), "Over the boat: landing")

        if state == "LAND":
            HAL.land()                     # blocking: returns once landed
            state = "DONE"
            pos = HAL.get_position()
            say(log, state, pos, yaw, len(survivors), "Landed. Castaways found: %d" % len(survivors))
            print("-" * 70)
            for i, s in enumerate(survivors):
                print("Castaway %d: x = %6.2f m, y = %6.2f m  ->  %s"
                      % (i + 1, s["x"], s["y"], local_to_gps_text(s["x"], s["y"])))
            print("-" * 70)
            target = None

        if state not in ("LAND", "DONE"):
            HAL.set_cmd_pos(target[0], target[1], SEARCH_ALTITUDE, yaw0)
            log.row(state, pos, yaw, len(survivors))

        WebGUI.showLeftImage(mission_map.render(pos, state, survivors, rejected, target))
        WebGUI.showImage(camera_view(img, detections, state))

        if state == "DONE":
            time.sleep(0.5)
        else:
            time.sleep(0.02)


cascade = None
run_mission()
