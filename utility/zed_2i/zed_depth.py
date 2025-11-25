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
depth = sl.Mat()

# Global mouse coordinates
mouse_x, mouse_y = -1, -1

print("Hover over the image to see depth value (press 'q' to quit, press 's' to save).")

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y

cv2.namedWindow("ZED Depth Viewer")
cv2.setMouseCallback("ZED Depth Viewer", mouse_callback)

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve color image and depth map
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

        frame = image.get_data()

        # Ensure a proper 8-bit 3-channel BGR image
        if frame.shape[2] == 4:  # ZED usually returns BGRA
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        display_frame = frame.copy()


        if 0 <= mouse_x < frame.shape[1] and 0 <= mouse_y < frame.shape[0]:
            err, depth_value = depth.get_value(mouse_x, mouse_y)
            if err == sl.ERROR_CODE.SUCCESS and np.isfinite(depth_value):
                text = f"Depth: {depth_value:.1f} cm"
                color = (0, 255, 0)
            else:
                text = "No depth"
                color = (0, 0, 255)

            # --- Draw overlay box ---
            font_scale = 0.6
            thickness = 2
            (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)

            # Rectangle background coordinates
            rect_x1, rect_y1 = mouse_x + 10, mouse_y - 30
            rect_x2, rect_y2 = rect_x1 + text_w + 10, rect_y1 + text_h + 10

            # Ensure it stays within image bounds
            rect_x1 = max(0, rect_x1)
            rect_y1 = max(0, rect_y1)
            rect_x2 = min(frame.shape[1], rect_x2)
            rect_y2 = min(frame.shape[0], rect_y2)

            # Semi-transparent rectangle background
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (255, 255, 255), -1)
            display_frame = cv2.addWeighted(overlay, 0.6, display_frame, 0.4, 0, display_frame)

            # Text
            text_pos = (rect_x1 + 5, rect_y2 - 5)
            cv2.putText(display_frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

            # Highlight circle
            cv2.circle(display_frame, (mouse_x, mouse_y), 5, color, -1)

        cv2.imshow("ZED Depth Viewer", display_frame)

        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key == ord('q'):
            break

        # Save screenshot
        if key == ord('s'):
            # Use timestamp-based filename to avoid collisions
            filename = f"capture_{int(cv2.getTickCount())}.png"
            cv2.imwrite(filename, display_frame)
            print(f"Saved image: {filename}")


cv2.destroyAllWindows()
zed.close()
