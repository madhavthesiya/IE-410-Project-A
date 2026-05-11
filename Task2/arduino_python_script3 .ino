/*
 * Braccio Pick-and-Place — Arduino (Fixed)
 *
 * Serial protocol (115200 baud):
 *   RECEIVE  "G:xP,yP,zP,xL,yL,zL\n"  → full pick-and-place cycle (mm)
 *   RECEIVE  "H\n"                      → go to home position
 *   SEND     "Done\n"                   → motion complete confirmation
 *
 * IK is computed on the Arduino side (standard geometric IK).
 * Python only sends Cartesian coordinates — no duplicate IK calculation.
 *
 * Fixes applied:
 *   1. Added "H\n" home command handler (was missing — Python could not home)
 *   2. Added line.trim() to strip \r from Windows-style line endings
 *   3. Added servo angle clamping to valid Braccio limits
 *   4. Added Dp = max(Dp, 1.0) guard to prevent IK sqrt/acos domain errors
 *   5. Approach now opens gripper BEFORE descending (logical pick sequence)
 *   6. Sends "Ready\n" on startup so Python knows Arduino has booted
 */

#include <Servo.h>
#include "BraccioRobot.h"
#include <math.h>

#define PI 3.14159265358979323846

// ── Link parameters (mm) ───────────────────────────────────────────────────
const double l0 = 71.5;    // base-to-shoulder height
const double L1 = 125.0;   // upper arm length
const double L2 = 125.0;   // forearm length
const double L3 = 192.0;   // wrist-to-gripper tip length

// Desired end-effector pitch (rad): -90° = pointing straight down
const double phi = -PI / 2.0;

Position armPosition;

// ── Helpers ────────────────────────────────────────────────────────────────
void goHome() {
  armPosition.set(90, 90, 90, 90, 90, 73);   // safe neutral pose, gripper closed
  BraccioRobot.moveToPosition(armPosition, 100);
  delay(500);
}

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  BraccioRobot.init();
  goHome();
  Serial.println("Ready");   // FIX: tell Python the arm has booted
}

// ── Serial parser ──────────────────────────────────────────────────────────
/*
 * Returns true only when a valid "G:" pick-and-place command was received.
 * Handles "H\n" home command inline (sends "Done" and returns false).
 */
bool readCommand(double &xP, double &yP, double &zP,
                 double &xL, double &yL, double &zL) {
  if (!Serial.available()) return false;

  String line = Serial.readStringUntil('\n');
  line.trim();   // FIX: strip \r so "G:" is not missed on Windows

  // ── Home command ─────────────────────────────────────────────────────
  if (line == "H") {
    goHome();
    Serial.println("Done");
    return false;
  }

  // ── Pick-and-place command ────────────────────────────────────────────
  if (line.length() < 2 || line.charAt(0) != 'G' || line.charAt(1) != ':')
    return false;

  String payload = line.substring(2);   // strip "G:"
  double vals[6];
  int    idx = 0;
  char   buf[80];
  payload.toCharArray(buf, sizeof(buf));
  char *tok = strtok(buf, ",");
  while (tok && idx < 6) {
    vals[idx++] = atof(tok);
    tok = strtok(NULL, ",");
  }
  if (idx != 6) return false;

  xP = vals[0]; yP = vals[1]; zP = vals[2];
  xL = vals[3]; yL = vals[4]; zL = vals[5];
  return true;
}

// ── Inverse Kinematics ─────────────────────────────────────────────────────
/*
 * Standard geometric IK for a 3-link planar arm with fixed end-effector pitch.
 * All outputs are in DEGREES ready for Braccio servos.
 */
void computeIK(double x, double y, double z,
               double &t1, double &t2, double &t3, double &t4) {
  // Base rotation
  t1 = atan2(y, x);

  // Effective reach (subtract tool length to reduce to 2-link problem)
  double r  = sqrt(x * x + y * y);
  double s  = z - l0;
  double D  = sqrt(r * r + s * s);
  double Dp = D - L3;
  Dp = max(Dp, 1.0);   // FIX: prevent domain error when target is too close

  // Elbow angle via law of cosines
  double cosA = (L1 * L1 + L2 * L2 - Dp * Dp) / (2.0 * L1 * L2);
  cosA = constrain(cosA, -1.0, 1.0);
  double alpha = acos(cosA);
  t3 = PI - alpha;

  // Shoulder angle
  double beta = atan2(s, r);
  double cosG = (L1 * L1 + Dp * Dp - L2 * L2) / (2.0 * L1 * Dp);
  cosG = constrain(cosG, -1.0, 1.0);
  double gamma = acos(cosG);
  t2 = beta + gamma;

  // Wrist angle to maintain constant pitch
  t4 = phi - (t2 + t3);

  // Convert all to degrees
  t1 *= 180.0 / PI;
  t2 *= 180.0 / PI;
  t3 *= 180.0 / PI;
  t4 *= 180.0 / PI;
}

// ── Motion primitive ───────────────────────────────────────────────────────
void moveToXYZ(double x, double y, double z, bool closeGripper) {
  double t1, t2, t3, t4;
  computeIK(x, y, z, t1, t2, t3, t4);

  // Map IK angles to Braccio servo space and clamp to safe limits
  //   FIX: constrain() applied to every joint to avoid mechanical binding
  int b  = constrain((int)round(180.0 - t1),  0,  180);   // base
  int s  = constrain((int)round(t2),           15, 165);   // shoulder
  int e  = constrain((int)round(t3),           0,  180);   // elbow
  int w  = constrain((int)round(180.0 - t4),   0,  180);   // wrist pitch
  int wr = 90;                                              // wrist rotation (neutral)
  int g  = closeGripper ? 73 : 0;                           // gripper

  armPosition.set(b, s, e, w, wr, g);
  BraccioRobot.moveToPosition(armPosition, 100);
  delay(300);
}

// ── Pick-and-place sequence ────────────────────────────────────────────────
/*
 * FIX: Gripper is opened BEFORE descending onto the object.
 *      Original code approached with gripper closed then opened while touching —
 *      this could knock the object before grasping.
 *
 * Sequence:
 *   1. Hover 100 mm above pick, gripper OPEN  (safe approach)
 *   2. Descend to pick height, gripper OPEN
 *   3. Close gripper (grasp)
 *   4. Lift 100 mm
 *   5. Hover 100 mm above place, gripper CLOSED
 *   6. Descend to place height
 *   7. Open gripper (release)
 *   8. Retract 100 mm
 *   9. Return to home
 */
void pickAndPlace(double xP, double yP, double zP,
                  double xL, double yL, double zL) {
  moveToXYZ(xP, yP, zP + 100.0, false);  // approach above pick, open
  moveToXYZ(xP, yP, zP,         false);  // descend to pick, open
  moveToXYZ(xP, yP, zP,         true);   // close gripper — grasp
  moveToXYZ(xP, yP, zP + 100.0, true);   // lift

  moveToXYZ(xL, yL, zL + 100.0, true);   // move above place
  moveToXYZ(xL, yL, zL,         true);   // descend to place
  moveToXYZ(xL, yL, zL,         false);  // open gripper — release
  moveToXYZ(xL, yL, zL + 100.0, false);  // retract up

  goHome();
}

// ── Main loop ──────────────────────────────────────────────────────────────
void loop() {
  double xP, yP, zP, xL, yL, zL;
  if (readCommand(xP, yP, zP, xL, yL, zL)) {
    pickAndPlace(xP, yP, zP, xL, yL, zL);
    Serial.println("Done");
  }
}
