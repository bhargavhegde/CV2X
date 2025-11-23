
# This function is written to use YOLO generate some ground truth verification
# author: Haosong Xiao
# requires the model file best.pt in the same folder
# one can download trained model from: https://buffalo.box.com/s/6xe3urkqnz4p5l5uggyf5wd5oyo2t6ix


import os
from ultralytics import YOLO

def Yolo_gt_HX(img, confidence_threshold=0.1):
    """
    Get YOLO bounding boxes for checkerboard images
    Args:
        img: Image from cv2.imread()
        confidence_threshold: Minimum confidence for detections (default: 0.1)
    Returns:
        list: List of bounding boxes as [x1, y1, x2, y2] format
    """
    # Get the directory of this script to find best.pt
    script_dir = os.path.dirname(os.path.abspath(__file__))
    trained_params_path = os.path.join(script_dir, "best.pt")
    model = YOLO(trained_params_path)
    results = model.predict(source=img, save=False, verbose=False)

    bounding_boxes = []

    # Extract bounding boxes with confidence filtering
    for r in results:
        if r.boxes is not None and len(r.boxes) > 0:
            for box in r.boxes:
                confidence = box.conf[0].item()
                print('current confidence', confidence)
                if confidence >= confidence_threshold:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    bounding_boxes.append([int(x1), int(y1), int(x2), int(y2)])

    # If no boxes were found after filtering, return a placeholder
    if not bounding_boxes:
        bounding_boxes.append([-1, -1, -1, -1])

    return bounding_boxes

def Yolo_window_adjustment(x1_yolo, y1_yolo, x2_yolo, y2_yolo, x1, y1, x2, y2):
    """
    Adjust the YOLO bounding box coordinates to the desired window coordinates.
    Args:
        x1_yolo, y1_yolo, x2_yolo, y2_yolo: YOLO bounding box coordinates
        x1, y1, x2, y2: Desired window coordinates
    Returns:
        list: Adjusted bounding box coordinates as [x1, y1, x2, y2]
    """
    x1_adjust = max(0, x1 - x1_yolo + 10)
    x2_adjust = max(0, x2_yolo - x2 + 10)
    y1_adjust = max(0, y1 - y1_yolo + 10)
    y2_adjust = max(0, y2_yolo - y2 + 10)

    if x1_adjust > 0:
        x1_adjusted = x1 - x1_adjust
        x2_adjusted = x2 - x1_adjust
        if y1_adjust > 0:
            y1_adjusted = y1 - y1_adjust
            y2_adjusted = y2 - y1_adjust
        elif y2_adjust > 0:
            y1_adjusted = y1 + y2_adjust
            y2_adjusted = y2 + y2_adjust
        else:
            y1_adjusted = y1
            y2_adjusted = y2
    elif x2_adjust > 0:
        x1_adjusted = x1 + x2_adjust
        x2_adjusted = x2 + x2_adjust
        if y1_adjust > 0:
            y1_adjusted = y1 - y1_adjust
            y2_adjusted = y2 - y1_adjust
        elif y2_adjust > 0:
            y1_adjusted = y1 + y2_adjust
            y2_adjusted = y2 + y2_adjust
        else:
            y1_adjusted = y1
            y2_adjusted = y2
    else:
        x1_adjusted = x1
        x2_adjusted = x2
        if y1_adjust > 0:
            y1_adjusted = y1 - y1_adjust
            y2_adjusted = y2 - y1_adjust
        elif y2_adjust > 0:
            y1_adjusted = y1 + y2_adjust
            y2_adjusted = y2 + y2_adjust
        else:
            y1_adjusted = y1
            y2_adjusted = y2

    return x1_adjusted, y1_adjusted, x2_adjusted, y2_adjusted
