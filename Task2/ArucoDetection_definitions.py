# -*- coding: utf-8 -*-
"""
ArucoDetection_definitions.py  —  ArUco utility functions.

Fixes applied vs original:
  1. getMarkerCenter_foam() completely rewritten — original called
     getMarkerCoordinates() four times (once per corner) and then used
     inconsistent indexing (pt0 vs pts1[0][0]).  New version reads the
     raw marker corner array directly via NumPy, which is simpler, faster,
     and has no IndexError risk when markers list is empty.
  2. draw_field() now returns found=False when any of the 4 pts[] slots
     is still None (happens if corner IDs arrive out of order).  Original
     would raise TypeError when trying to pass None to np.array().
"""

import cv2
import numpy as np


def getMarkerCoordinates(markers, ids, point=0):
    """
    Extract a specified corner from each detected marker.

    Parameters
    ----------
    markers : list of np.ndarray, each shape (1, 4, 2)
    ids     : list of int marker IDs (same length as markers)
    point   : corner index  0=top-left  1=top-right  2=bottom-right  3=bottom-left

    Returns
    -------
    corners : list of [x, y]  (one per marker)
    ids     : the same ids list passed in
    """
    corners = []
    for marker in markers:
        x = int(marker[0][point][0])
        y = int(marker[0][point][1])
        corners.append([x, y])
    return corners, ids


def getMarkerCenter_foam(markers):
    """
    Compute the pixel centroid of the first detected foam/object marker.

    FIX: Original called getMarkerCoordinates() four separate times and
    mixed pt0 (a plain [x,y]) with pts1[0][0] (nested list access), which
    was confusing and fragile.  New version uses NumPy mean over the 4
    corners of markers[0], which is both correct and crash-safe.

    Returns [[cx, cy]] or [[0, 0]] when no marker is detected.
    """
    if not markers:
        return [[0, 0]]

    # markers[0] has shape (1, 4, 2): one marker, four corners, x+y
    corners = markers[0][0]          # shape (4, 2)
    cx = int(np.mean(corners[:, 0]))
    cy = int(np.mean(corners[:, 1]))
    return [[cx, cy]]


def draw_corners(img, points):
    """Draw a green dot at each (x, y) point."""
    for (x, y) in points:
        cv2.circle(img, (x, y), 6, (0, 255, 0), -1)


def draw_numbers(img, corners, ids):
    """Draw the marker ID next to each corner point."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, (x, y) in enumerate(corners):
        cv2.putText(img, str(ids[i]), (x + 6, y + 6), font, 0.5, (0, 0, 0), 2)


def show_spec(img, corners):
    """Overlay the count of detected corners."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = f"{len(corners)} markers found."
    cv2.putText(img, text, (10, 20), font, 0.6, (0, 0, 255), 2)


def draw_field(img, corners, ids):
    """
    Fill the workspace quadrilateral with a semi-transparent gold overlay
    when all 4 boundary markers are present.

    FIX: Added a None-check on each pts[] slot.  If marker IDs arrive
    out of order before all 4 are seen, pts may contain None entries,
    which caused a TypeError in np.array() in the original code.

    Parameters
    ----------
    img     : source BGR frame
    corners : list of 4 [x, y] points (indexed 0-3, corresponding to IDs 1-4)
    ids     : list of 4 ints (expected to be [1, 2, 3, 4])

    Returns
    -------
    img_out : annotated frame
    found   : True only when all 4 non-None corners were drawn
    """
    if len(corners) != 4 or len(ids) != 4:
        return img, False

    pts = [None] * 4
    for idx, cid in enumerate(ids):
        if 1 <= cid <= 4:
            pts[cid - 1] = corners[idx]

    # FIX: abort if any slot is still None
    if any(p is None for p in pts):
        return img, False

    pts_np  = np.array(pts, dtype=np.int32)
    overlay = img.copy()
    cv2.fillPoly(overlay, [pts_np], (255, 215, 0))               # gold fill
    img_out = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
    cv2.polylines(img_out, [pts_np], True, (0, 200, 255), 2)     # orange border
    return img_out, True


def order_points(pts):
    """
    Order four corner points as: top-left, top-right, bottom-right, bottom-left.
    Required by four_point_transform() for a correct perspective warp.
    """
    rect = np.zeros((4, 2), dtype="float32")
    s    = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]    # TL: smallest x+y
    rect[2] = pts[np.argmax(s)]    # BR: largest  x+y
    diff    = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)] # TR: smallest y-x
    rect[3] = pts[np.argmax(diff)] # BL: largest  y-x
    return rect


def four_point_transform(image, pts):
    """
    Warp the quadrilateral region defined by pts into a rectangular
    bird's-eye view image.

    Parameters
    ----------
    image : BGR frame
    pts   : (4, 2) array of corner coordinates in any order

    Returns
    -------
    warped : rectified BGR image of the workspace
    """
    rect = order_points(pts.astype("float32"))
    tl, tr, br, bl = rect

    maxW = int(max(np.linalg.norm(br - bl), np.linalg.norm(tr - tl)))
    maxH = int(max(np.linalg.norm(tr - br), np.linalg.norm(tl - bl)))

    dst = np.array([
        [0,      0],
        [maxW-1, 0],
        [maxW-1, maxH-1],
        [0,      maxH-1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(image, M, (maxW, maxH))
