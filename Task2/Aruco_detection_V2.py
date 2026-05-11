#!/usr/bin/env python
"""
Aruco_detection_V2.py  —  Main ArUco-based vision pick-and-place control loop.

Camera: DroidCam app (Android or iPhone) over Wi-Fi.
  1. Install DroidCam on your phone.
  2. Open DroidCam — note the IP and Port shown on screen.
  3. Connect phone and PC to the SAME Wi-Fi network.
  4. Set DROIDCAM_IP below to the IP shown in the DroidCam app.

Controls (press in the 'workspace' window):
  p  →  trigger pick-and-place on the currently detected object
  q  →  quit

Fixes applied vs original:
  1. CRITICAL — Pixel-to-robot coordinate mapping had u and v SWAPPED.
       u = center x-pixel  →  must be normalised by frame WIDTH  (for robot X)
       v = center y-pixel  →  must be normalised by frame HEIGHT (for robot Y)
     Original used v/w for X and u/h for Y, mirroring every pick position.

  2. CRITICAL — 'found' flag was always True from frame 1.
     Original hardcoded corner_ids = [1,2,3,4] so draw_field() always saw
     4 IDs and returned found=True, even when using the fallback init_locs.
     Fix: track which workspace marker IDs have actually been detected.
     'workspace_ready' is only True after all 4 real corners are observed.

  3. Replaced keyboard.is_pressed('p') with cv2.waitKey().
     The 'keyboard' module requires root privileges on Linux/Mac and raises
     a PermissionError.  cv2.waitKey() works cross-platform without elevation.

  4. Fixed pick debounce: a 60-frame cooldown prevents a single key press
     from triggering multiple sequential pick commands.

  5. current_center_Corner was never updated (always returned [[0,0]]).
     Now updated each frame from the detected centroid.

  6. Removed 'import keyboard' dependency entirely.

  7. Added on-screen status text showing workspace readiness and pick state.
"""

import time
import cv2
import numpy as np
from ArucoDetection_definitions import (
    getMarkerCoordinates,
    getMarkerCenter_foam,
    draw_corners,
    draw_field,
    four_point_transform,
)
import braccio_control_python

# =============================================================================
# DroidCam configuration — edit ONLY these two values
# =============================================================================
DROIDCAM_IP   = "10.120.215.199"   # ← replace with IP shown in DroidCam app
DROIDCAM_PORT = "4747"             # default port (same for Android AND iPhone)
# =============================================================================

DROIDCAM_URL = f"http://{DROIDCAM_IP}:{DROIDCAM_PORT}/mjpegfeed"

# ArUco dictionaries
DICT_WORKSPACE = "DICT_4X4_50"   # boundary markers (IDs 1–4)
DICT_OBJECT    = "DICT_6X6_50"   # object / foam marker

ARUCO_DICT = {
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":        cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":        cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000":       cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":        cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000":       cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50":         cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100":        cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250":        cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000":       cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50":         cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100":        cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250":        cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000":       cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# Robot workspace physical dimensions (mm) — calibrate to match your setup
WORKSPACE_WIDTH_MM  = 600   # full X span in robot frame
WORKSPACE_HEIGHT_MM = 300   # full Y span in robot frame
WORKSPACE_X_OFFSET  = 300   # pixels at left edge map to X = -300 mm

# ── State ──────────────────────────────────────────────────────────────────
# Fallback pixel corners used before real workspace markers are detected.
_init_locs            = [[10, 400], [400, 400], [400, 10], [10, 10]]
current_square_points = [pt[:] for pt in _init_locs]  # deep copy

# FIX: track which workspace IDs have actually been seen
markers_seen          = set()

# FIX: keep last known centroid so it persists between frames
current_center = [[0, 0]]


# ── Helpers ────────────────────────────────────────────────────────────────

def get_markers(frame, detector):
    """Detect ArUco markers using OpenCV 4.7+ ArucoDetector API."""
    bboxs, ids, _ = detector.detectMarkers(frame)
    ids_list = [int(i[0]) for i in ids] if ids is not None else []
    return bboxs, ids_list


def connect_droidcam():
    """Open DroidCam MJPEG stream and verify a frame can be read."""
    print(f"[INFO] Connecting to DroidCam at {DROIDCAM_URL} ...")
    cap = cv2.VideoCapture(DROIDCAM_URL)
    time.sleep(2)   # give IP stream time to negotiate

    if not cap.isOpened():
        print("[ERROR] Could not open DroidCam stream. Check:")
        print(f"          • DROIDCAM_IP is correct (currently '{DROIDCAM_IP}')")
        print(f"          • DroidCam app is open and running on your phone")
        print(f"          • Phone and PC are on the SAME Wi-Fi network")
        print(f"          • Port {DROIDCAM_PORT} is not blocked by Windows Firewall")
        return None

    ret, frame = cap.read()
    if not ret or frame is None:
        print("[ERROR] DroidCam opened but could not read a frame.")
        print("        Try restarting the DroidCam app on your phone.")
        cap.release()
        return None

    h, w = frame.shape[:2]
    print(f"[INFO] DroidCam connected successfully — resolution {w}×{h}")
    return cap


# ── Main loop ──────────────────────────────────────────────────────────────

def main():
    global current_square_points, markers_seen, current_center

    print("[INFO] Initialising ArUco detectors...")

    dict1     = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[DICT_WORKSPACE])
    detector1 = cv2.aruco.ArucoDetector(dict1, cv2.aruco.DetectorParameters())

    dict2     = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[DICT_OBJECT])
    detector2 = cv2.aruco.ArucoDetector(dict2, cv2.aruco.DetectorParameters())

    cap = connect_droidcam()
    if cap is None:
        return

    print("[INFO] Press 'p' in the workspace window to pick, 'q' to quit.")

    warped        = None
    pick_cooldown = 0   # frames remaining before next pick is allowed

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Lost frame — retrying...")
            time.sleep(0.1)
            continue

        # ── 1. Workspace boundary detection ──────────────────────────────
        w_markers, w_ids = get_markers(frame, detector1)
        frame_clean      = frame.copy()
        corners, cids    = getMarkerCoordinates(w_markers, w_ids, point=0)

        # Update only the slots for markers we actually detected this frame
        for i, cid in enumerate(cids):
            if 1 <= cid <= 4:
                current_square_points[cid - 1] = corners[i]
                markers_seen.add(cid)

        # FIX: workspace is "ready" only once all 4 real corners are known
        workspace_ready = markers_seen >= {1, 2, 3, 4}

        # draw_field now checks for None entries internally (fixed in definitions)
        frame_viz, quad_ok = draw_field(frame, current_square_points, [1, 2, 3, 4])
        found = quad_ok and workspace_ready

        # Status overlay
        if found:
            status_text  = "Workspace READY — press 'p' to pick"
            status_color = (0, 220, 0)
        else:
            status_text  = f"Scanning for workspace markers ({len(markers_seen)}/4)"
            status_color = (0, 100, 255)
        cv2.putText(frame_viz, status_text, (10, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.imshow("workspace", frame_viz)

        # ── 2. Object detection in warped bird's-eye view ─────────────────
        if found:
            warped = four_point_transform(
                frame_clean, np.array(current_square_points, dtype="float32")
            )
            foam_markers, _ = get_markers(warped, detector2)
            detected_center = getMarkerCenter_foam(foam_markers)

            # FIX: persist last known centroid so 'p' always has a valid target
            if detected_center != [[0, 0]]:
                current_center = detected_center

            draw_corners(warped, current_center)

            # Show object location in mm on screen
            u_px, v_px = current_center[0]
            h_w, w_w   = warped.shape[:2]
            x_mm = int((u_px / w_w) * WORKSPACE_WIDTH_MM) - WORKSPACE_X_OFFSET
            y_mm = int((v_px / h_w) * WORKSPACE_HEIGHT_MM)
            cv2.putText(warped, f"Object: x={x_mm} y={y_mm} mm",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("object view", warped)

        # ── 3. Key handling (FIX: cv2.waitKey instead of keyboard module) ──
        key = cv2.waitKey(1) & 0xFF

        # Pick trigger
        if key == ord('p') and found and warped is not None and pick_cooldown == 0:
            h_w, w_w = warped.shape[:2]

            # FIX: u is x-pixel → normalise by WIDTH  for robot X
            #      v is y-pixel → normalise by HEIGHT for robot Y
            u_px, v_px = current_center[0]
            x_raw = int((u_px / w_w) * WORKSPACE_WIDTH_MM) - WORKSPACE_X_OFFSET
            y_raw = int((v_px / h_w) * WORKSPACE_HEIGHT_MM)

            x_corr, y_corr = braccio_control_python.camera_compensation(x_raw, y_raw)

            print(f"[PICK] pixel=({u_px},{v_px})  "
                  f"raw=({x_raw},{y_raw}) mm  "
                  f"corrected=({x_corr},{y_corr}) mm")

            braccio_control_python.pick_up(x_corr, y_corr)

            # FIX: cooldown prevents double-trigger from key held down
            pick_cooldown = 60

        # Count down cooldown every frame
        if pick_cooldown > 0:
            pick_cooldown -= 1

        # Quit
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ── Entry point ────────────────────────────────────────────────────────────

if __name__ == "__main__":
    braccio_control_python.home()   # move arm to safe start position
    main()
