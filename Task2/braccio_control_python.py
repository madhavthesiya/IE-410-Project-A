# -*- coding: utf-8 -*-
"""
braccio_control_python.py  —  Braccio arm high-level motion control over serial.

Protocol (must match arduino_python_script3.ino):
  SEND  "G:xP,yP,zP,xL,yL,zL\\n"  → full pick-and-place (all values in mm)
  SEND  "H\\n"                      → move to home position
  RECV  "Done\\n"                   → motion complete confirmation

Fixes applied vs original:
  1. CRITICAL — Removed all angle-based "P" commands; now sends Cartesian "G:"
     coordinates so the protocol matches what the Arduino actually reads.
     (Original sent servo angles with "P" prefix; Arduino only understood "G:")
  2. Removed solverNNA / backlash-compensation — IK now runs on Arduino only,
     eliminating duplicated (and incorrect) IK calculations on the Python side.
  3. Removed prev_teta.txt dependency — no more FileNotFoundError on first run.
  4. Fixed division-by-zero in camera_compensation() by restructuring the
     perspective formula as  correction = coordinate * (h_foam / cam_z),
     which has a guaranteed non-zero denominator (cam_z is a physical constant).
  5. Added _wait_for_ready() to block until Arduino sends "Ready" after boot,
     preventing commands being sent before the arm has initialised.
  6. Added pick_cooldown in the caller (Aruco_detection_V2.py) to avoid
     double-triggering; this module just executes whatever it receives.
"""

import serial
import time

# ── Fixed deposit location in robot frame (mm) ─────────────────────────────
# Adjust to wherever "Point B" is physically located on your bench.
PLACE_X = 310.0
PLACE_Y =  95.0
PLACE_Z =  50.0   # height above table surface at the deposit point

# Height at which the gripper is at the object surface when picking (mm).
# Measure physically: distance from robot base plane to top of object.
PICK_Z = 10.0

# ── Serial connection ───────────────────────────────────────────────────────
# Change 'COM12' to your port (e.g. '/dev/ttyUSB0' on Linux / '/dev/cu.usbmodem...' on Mac).
SERIAL_PORT = 'COM12'
BAUD_RATE   = 115200

try:
    arm = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
    print(f"[INFO] Opened {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Could not open serial port '{SERIAL_PORT}': {e}")
    arm = None


# ── Internal helpers ────────────────────────────────────────────────────────

def _wait_for_ready(timeout_s=15):
    """
    Block until the Arduino sends "Ready\\n" (emitted in setup() after init).
    This prevents commands being sent before servos have initialised.
    """
    if arm is None:
        return
    print("[INFO] Waiting for Arduino 'Ready'...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if arm.in_waiting:
            line = arm.readline().decode(errors='ignore').strip()
            if line == "Ready":
                print("[INFO] Arduino is ready.")
                return
            # Print any other boot messages for debugging
            if line:
                print(f"[Arduino] {line}")
    print("[WARN] Timed out waiting for 'Ready' — proceeding anyway.")


def _wait_done(timeout_s=90):
    """
    Block until Arduino sends "Done\\n" confirming motion is complete.
    Returns True on success, False on timeout.
    """
    if arm is None:
        return False
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if arm.in_waiting:
            line = arm.readline().decode(errors='ignore').strip()
            if line == "Done":
                return True
            if line:
                print(f"[Arduino] {line}")
    print("[WARN] Timed out waiting for 'Done'.")
    return False


def _send(cmd: str):
    """Encode and send a command string (must already include \\n)."""
    if arm is None:
        print(f"[ERROR] No arm connection — skipping command: {cmd.strip()}")
        return
    arm.write(cmd.encode())
    print(f"[CMD] {cmd.strip()}")


# ── Public API ──────────────────────────────────────────────────────────────

def home():
    """
    Send home command.  Arduino moves to the neutral rest pose and confirms.
    FIX: original used angle-based write_arduino() which Arduino never understood.
    """
    _send("H\n")
    _wait_done()


def pick_up(x: float, y: float):
    """
    Execute a full pick-and-place cycle.

    Parameters
    ----------
    x, y : float
        Pick position in the robot frame (mm), after camera compensation.
        PICK_Z (global) is used for the z-height at the pick surface.
        PLACE_X / PLACE_Y / PLACE_Z (globals) define the deposit location.

    The Arduino handles:
      • IK computation for every waypoint
      • Approach / descend / grasp / lift / transport / release / retract / home
    Python simply sends the Cartesian coordinates and waits for "Done".

    FIX: original sent "P<angles>,200\\n" which Arduino never parsed.
    """
    cmd = (
        f"G:{x:.1f},{y:.1f},{PICK_Z:.1f},"
        f"{PLACE_X:.1f},{PLACE_Y:.1f},{PLACE_Z:.1f}\n"
    )
    _send(cmd)
    ok = _wait_done(timeout_s=120)
    if ok:
        print("[INFO] Pick-and-place complete.")
    else:
        print("[WARN] Motion may not have finished — check arm state.")


def camera_compensation(x_coord: int, y_coord: int):
    """
    Correct pixel-centroid coordinates for perspective foreshortening
    caused by the camera viewing the workspace at an angle from above.

    The correction is derived from similar triangles:
        apparent_error = real_height * (distance_from_camera / camera_height)

    FIX: original formula was  h_foam / (cam_z / coord)  which equals
         h_foam * coord / cam_z  but crashes with ZeroDivisionError when
         coord == 0.  Restructured to  coord * scale  where scale = h_foam / cam_z,
         which is always safe because cam_z is a physical constant > 0.

    Parameters  (all in mm, measured in robot frame)
    ----------
    x_coord, y_coord : int
        Object centroid mapped from pixel space to robot mm space
        (before perspective correction).

    Returns
    -------
    (x_final, y_final) : (int, int)
        Perspective-corrected robot frame coordinates (mm).
    """
    h_foam  = 80                  # height of the foam object being picked (mm)
    cam_pos = [480, 150, 880]     # camera position [x, y, z] in robot frame (mm)
    offset  = 300                 # half-width of the robot workspace (mm)

    # Re-centre x relative to workspace midpoint
    x_adj = (offset - x_coord) + (cam_pos[0] - offset)

    # Perspective scale factor — safe denominator (cam_pos[2] is always > 0)
    scale = h_foam / cam_pos[2]   # dimensionless, typically ~0.09

    # Apply correction proportional to how far the point is from the camera axis
    x_corr = x_adj * (1.0 - scale)   # equivalent to x_adj - x_adj*scale

    if y_coord < cam_pos[1]:
        # Object is between camera and centre → foreshortened → subtract
        y_corr = y_coord * (1.0 - scale)
    else:
        # Object is beyond centre away from camera → add
        y_corr = y_coord * (1.0 + scale)

    # Un-centre x back to workspace coordinates
    x_final = offset - (x_corr - (cam_pos[0] - offset))

    return int(x_final), int(y_corr)


# ── Initialisation on import ────────────────────────────────────────────────
# Wait for the Arduino to finish booting before any command is issued.
if arm is not None:
    _wait_for_ready()
