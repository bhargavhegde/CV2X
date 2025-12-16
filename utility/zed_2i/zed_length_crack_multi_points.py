import cv2
import numpy as np
import pyzed.sl as sl
import json

# ---------------- ZED init ----------------
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.CENTIMETER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED")
    exit()

runtime_params = sl.RuntimeParameters()
image = sl.Mat()
depth_mat = sl.Mat()   # single-channel depth
# ------------------------------------------------

# ---------------- state ----------------
frozen = False
frozen_image = None     # RGB image (BGR)
frozen_depth = None     # numpy float32 depth (same units as zed coordinate_units)
poly2d = []
poly3d = []
drawing = False
neighbor_search_radius = 3   # window radius to search for valid depth (change if needed)

# ----------------- helper: get intrinsics robustly -----------------
def get_intrinsics(zed):
    info = zed.get_camera_information()
    # try likely attribute paths
    try:
        left = info.camera_configuration.calibration_parameters.left_cam
        fx = left.fx
        fy = left.fy
        cx = left.cx
        cy = left.cy
        return fx, fy, cx, cy
    except Exception:
        # fallback to different path names if needed
        try:
            cal = info.calibration_parameters.left_cam
            return cal.fx, cal.fy, cal.cx, cal.cy
        except Exception:
            raise RuntimeError("Cannot read camera intrinsics from ZED camera info")

# ---------- helper: robust depth read with neighborhood median ----------
def get_valid_depth(depth_np, x, y, radius=3):
    h, w = depth_np.shape
    if x < 0 or x >= w or y < 0 or y >= h:
        return None
    # direct read
    val = float(depth_np[y, x])
    if np.isfinite(val) and val > 0:
        return val
    # neighborhood median
    x0 = max(0, x - radius)
    x1 = min(w, x + radius + 1)
    y0 = max(0, y - radius)
    y1 = min(h, y + radius + 1)
    window = depth_np[y0:y1, x0:x1].ravel()
    window_valid = window[np.isfinite(window) & (window > 0)]
    if window_valid.size == 0:
        return None
    return float(np.median(window_valid))

# ---------- pixel+depth -> 3D ----------
def pixel_to_3d(u, v, depth_val, fx, fy, cx, cy):
    # depth_val in same units as intrinsics (here cm)
    Z = depth_val
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z], dtype=np.float32)

# ---------------- mouse callback ----------------
def mouse_cb(event, x, y, flags, param):
    global drawing, frozen, poly2d, poly3d, frozen_depth

    if not frozen:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True

    if drawing and (event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_MOUSEMOVE):
        # try to obtain depth from frozen_depth numpy array
        d = get_valid_depth(frozen_depth, x, y, radius=neighbor_search_radius)
        if d is not None:
            p3 = pixel_to_3d(x, y, d, fx, fy, cx, cy)
            # avoid recording points too close together in pixel space (reduces duplicates)
            if len(poly2d) == 0 or (abs(poly2d[-1][0] - x) + abs(poly2d[-1][1] - y) > 2):
                poly2d.append((x, y))
                poly3d.append(p3)
                print(f"Added point {len(poly3d)}: pixel=({x},{y}) depth={d:.2f} -> 3D={p3}")
        else:
            # optional: show feedback; we avoid flooding console
            pass

    if event == cv2.EVENT_LBUTTONUP:
        drawing = False

# ---------- length calc ----------
def compute_length_3d(pts3d):
    if len(pts3d) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts3d)):
        total += np.linalg.norm(pts3d[i] - pts3d[i-1])
    return float(total)

# ---------- main loop ----------
fx, fy, cx, cy = get_intrinsics(zed)
print(f"Camera intrinsics fx={fx}, fy={fy}, cx={cx}, cy={cy}")

intrinsics = {
    "fx": float(fx),
    "fy": float(fy),
    "cx": float(cx),
    "cy": float(cy)
}

with open("camera_intrinsics.json", "w") as f:
    json.dump(intrinsics, f, indent=4)

print("Saved intrinsics to camera_intrinsics.json")

cv2.namedWindow("ZED Crack Measurement")
cv2.setMouseCallback("ZED Crack Measurement", mouse_cb)

print("""
Controls:
  f = Freeze current frame and enter Draw Mode
  r = Return to Live Mode (reset)
  c = Compute crack length
  u = Undo last segment
  s = Save annotated image
  q = Quit
""")

while True:
    if not frozen:
        # live mode: retrieve image + depth map
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # retrieve single-channel depth (meters or cm depending on units set earlier)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)   # DEPTH gives Z only

            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            display = frame.copy()
            cv2.putText(display, "Live - press 'f' to freeze", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("ZED Crack Measurement", display)
            key = cv2.waitKey(1) & 0xFF
        else:
            # grab failed; small wait
            key = cv2.waitKey(10) & 0xFF
    else:
        # frozen mode: show frozen_image with overlays
        display = frozen_image.copy()
        # draw polyline
        if len(poly2d) >= 2:
            pts = np.array(poly2d, dtype=np.int32)
            cv2.polylines(display, [pts], False, (0,0,255), 2)
        # draw points
        for i, (px, py) in enumerate(poly2d):
            cv2.circle(display, (px, py), 4, (0,255,0), -1)
            cv2.putText(display, f"{i+1}", (px+6, py-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        # show point count and length
        cv2.putText(display, f"Points: {len(poly3d)}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
        cv2.putText(display, f"Length: {compute_length_3d(poly3d):.2f} cm", (10,65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

        cv2.imshow("ZED Crack Measurement", display)
        key = cv2.waitKey(1) & 0xFF

    # ---------------- keyboard actions ----------------
    if key == ord('q'):
        break

    if key == ord('f') and not frozen:
        # freeze current frame and depth map
        # ensure we have the last grabbed data: image and depth_mat must be filled
        frozen = True
        frozen_image = image.get_data().copy()
        if frozen_image.shape[2] == 4:
            frozen_image = cv2.cvtColor(frozen_image, cv2.COLOR_BGRA2BGR)

        # convert depth_mat to numpy 2D array
        depth_arr = depth_mat.get_data()   # should be HxW single channel float
        # sometimes ZED returns depth in meters depending on units; we set coordinate_units to cm above
        frozen_depth = np.array(depth_arr, copy=True)  # guarantee numpy array copy

        poly2d = []
        poly3d = []
        print("Frame frozen. Draw mode ACTIVE.")

    if key == ord('r') and frozen:
        frozen = False
        frozen_image = None
        frozen_depth = None
        poly2d = []
        poly3d = []
        print("Returned to live mode.")

    if key == ord('c') and frozen:
        if len(poly3d) >= 2:
            length_cm = compute_length_3d(poly3d)
            print(f"Crack length = {length_cm:.2f} cm")
        else:
            print("Need at least two 3D points to compute length.")

    if key == ord('u') and frozen:
        if len(poly2d) > 0:
            poly2d.pop()
            poly3d.pop()
            print("Undid last point.")
        else:
            print("Nothing to undo.")

    if key == ord('s') and frozen:
        fname = f"crack_{int(cv2.getTickCount())}.png"
        # annotate again before saving
        save_img = frozen_image.copy()
        if len(poly2d) >= 2:
            cv2.polylines(save_img, [np.array(poly2d, dtype=np.int32)], False, (0,0,255), 2)
        for i,(px,py) in enumerate(poly2d):
            cv2.circle(save_img, (px,py), 4, (0,255,0), -1)
            cv2.putText(save_img, f"{i+1}", (px+6, py-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.putText(save_img, f"Length: {compute_length_3d(poly3d):.2f} cm", (10,70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)
        cv2.imwrite(fname, save_img)
        print(f"Saved {fname}")

# cleanup
cv2.destroyAllWindows()
zed.close()
