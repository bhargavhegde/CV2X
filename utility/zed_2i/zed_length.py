import cv2
import numpy as np
import pyzed.sl as sl

# Initialize ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.CENTIMETER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED.")
    exit()

runtime_params = sl.RuntimeParameters()

image = sl.Mat()
point_cloud = sl.Mat()

# Storage for selected points
points = []  # Will store [((x,y), (X,Y,Z)), ((x,y), (X,Y,Z))]

def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        err, point3D = point_cloud.get_value(x, y)
        if err == sl.ERROR_CODE.SUCCESS and np.isfinite(point3D[2]):
            print(f"Point selected: Pixel ({x},{y}) → 3D {point3D}")
            points.append(((x, y), point3D))

            if len(points) > 2:
                points = points[-2:]  # Keep only last 2 points

cv2.namedWindow("ZED Distance Viewer")
cv2.setMouseCallback("ZED Distance Viewer", mouse_callback)

print("Click two points to measure distance (press 's' to save image, 'q' to quit).")

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        frame = image.get_data()

        # ---- FIX: Convert ZED BGRA → BGR to avoid corrupted saved images ----
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        display_frame = frame.copy()

        # Draw selected points
        for i, ((px, py), _) in enumerate(points):
            cv2.circle(display_frame, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(display_frame, f"P{i+1}", (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # If two points selected, compute and show distance
        if len(points) == 2:
            (_, p1), (_, p2) = points
            p1 = np.array(p1[:3])
            p2 = np.array(p2[:3])

            distance_cm = np.linalg.norm(p1 - p2)
            text = f"Distance: {distance_cm:.2f} cm"

            # Overlay box
            (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(display_frame, (10, 10), (20 + w, 20 + h), (255, 255, 255), -1)

            cv2.putText(display_frame, text, (15, 15 + h),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("ZED Distance Viewer", display_frame)

        # ----------- KEY HANDLING -------------
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord('s'):
            filename = f"distance_capture_{int(cv2.getTickCount())}.png"
            cv2.imwrite(filename, display_frame)
            print(f"Saved image: {filename}")
        # --------------------------------------

cv2.destroyAllWindows()
zed.close()
