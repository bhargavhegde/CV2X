# This code is written for generating quantification results for the offline tuning. 
# Written with the help of Copilot.
# Author: Haosong Xiao
# Date: 2024-09-23

# library imports
import matplotlib.pyplot as plt
import numpy as np
import os
from quantification.pixel_conversion import Cameras_HX
from quantification.YOLO_gt_HX import Yolo_gt_HX



script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, 'quantification', 'config', 'camera.json')
target_image_path = os.path.join(script_dir, 'data', 'Test_20251117_010', 'image_folder', 'frame_1763395962757_1.png')
sync_report_path = os.path.join(script_dir, 'data', 'Test_20251117_010', 'sync_report_log.txt') # this tells which crop message was synced
crop_log_path = os.path.join(script_dir, 'data', 'Test_20251117_010', 'detection_record_1_log.txt') 
crack_location_path = os.path.join(script_dir, 'data', 'Test_20251117_010', 'camera_parameters.txt')



def get_box(width, height, u, v, box_size=512):
    half_box = box_size // 2
    # print('u,v:', u,v)
    # Initial box centered at (u, v)
    x1 = u - half_box
    x2 = u + half_box
    y1 = v - half_box
    y2 = v + half_box
    # print('Initial box:', x1, x2, y1, y2)
    # Adjust box if it goes out of image boundaries
    if x1 < 0:
        x1 = 0
        x2 = box_size
    elif x2 > width:
        x2 = width
        x1 = width - box_size
    # print('After width adjustment box:', x1, x2, y1, y2)
    if y1 < 0:
        y1 = 0
        y2 = box_size
        # print('After height adjustment box:', x1, x2, y1, y2)
    elif y2 > height:
        # print('y2 before adjustment:', y2)
        # print('height:', height)
        y2 = height
        y1 = height - box_size
        # print('After height adjustment box:', x1, x2, y1, y2)

    # Ensure the box stays within bounds
    # print('Adjusted box:', x1, x2, y1, y2)
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(width, x2)
    y2 = min(height, y2)
    # print('Adjusted box:', x1, x2, y1, y2)
    return int(x1), int(x2), int(y1), int(y2)

def edge_projection(crop_center, Yolo_center, width, height):
    u, v = crop_center
    u_yolo, v_yolo = Yolo_center
    # print("Crop center (u,v):", (u,v))
    # print("YOLO center (u_yolo,v_yolo):", (u_yolo, v_yolo))
    dx = u_yolo - u
    dy = v_yolo - v
    # print("dx, dy:", (dx, dy))
    if dx == 0:
        edge_x = u
        edge_y = v + (height / 2) * (1 if dy > 0 else -1)
    elif dy == 0:
        edge_x = u + (width / 2) * (1 if dx > 0 else -1)
        edge_y = v
    else:
        scale_x = (width / 2) / abs(dx)
        scale_y = (height / 2) / abs(dy)
        scale = min(scale_x, scale_y)
        edge_x = u + dx * scale
        edge_y = v + dy * scale

    return (edge_x, edge_y)

def finding_the_scores(YOLO_box, CROP_box, edge_coord):
    x1, y1, x2, y2 = CROP_box
    x1_yolo, y1_yolo, x2_yolo, y2_yolo = YOLO_box

    # center points
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2
    u_yolo = (x1_yolo + x2_yolo) / 2
    v_yolo = (y1_yolo + y2_yolo) / 2

    # check if YOLO box is within CROP box
    x1_inter = max(0, min(x2, x2_yolo) - max(x1, x1_yolo))
    y1_inter = max(0, min(y2, y2_yolo) - max(y1, y1_yolo))
    intersection_area = x1_inter * y1_inter
    yolo_area = (x2_yolo - x1_yolo) * (y2_yolo - y1_yolo)
    overlapping_rate = intersection_area / yolo_area if yolo_area > 0 else 0
   
    # check how close the YOLO center to the CROP center
    u_edge, v_edge = edge_coord
    if u_edge == u and v_edge == v:
        alignment_rate = 1.0  # Perfect alignment if edge coincides with crop center
    elif u_yolo < x1 or u_yolo > x2 or v_yolo < y1 or v_yolo > y2:
        alignment_rate = 0.0  # No alignment if YOLO center is outside the crop box
    else:
        # compute the length from edge to yolo center
        edge_to_yolo = np.sqrt((u_edge - u_yolo)**2 + (v_edge - v_yolo)**2)
        # compute the length from edge to crop center
        edge_to_crop = np.sqrt((u_edge - u)**2 + (v_edge - v)**2)
        alignment_rate = edge_to_yolo / edge_to_crop if edge_to_crop > 0 else 0
    print("overlapping_rate, alignment_rate:", (overlapping_rate, alignment_rate))
    overall_score = (overlapping_rate + alignment_rate) / 2
    return overall_score

def read_crack_location(crack_location_path):
    """Read crack latitude and longitude from camera_parameters.txt"""
    crack_lat = None
    crack_lon = None
    
    with open(crack_location_path, 'r') as f:
        content = f.read()
        # Parse the parameters from the format: "..., crack_lat: 42.992613, crack_lon: -78.793833, ..."
        for param in content.split(','):
            param = param.strip()
            if param.startswith('crack_lat:'):
                crack_lat = float(param.split(':')[1].strip())
            elif param.startswith('crack_lon:'):
                crack_lon = float(param.split(':')[1].strip())
    
    return crack_lat, crack_lon

def extract_timestamp_from_image(image_path):
    """Extract timestamp from image filename like 'frame_1763395962757_1.png'"""
    filename = os.path.basename(image_path)
    # Format: frame_TIMESTAMP_COUNT.png
    parts = filename.replace('.png', '').split('_')
    if len(parts) >= 2:
        timestamp = int(parts[1])
        return timestamp

def find_synced_crop_time(sync_report_path, image_timestamp):
    """Find the synced crop time that matches the image timestamp"""
    with open(sync_report_path, 'r') as f:
        for line in f:
            # Parse line: "Node start time: X, synced crop time: Y, synced img time: Z"
            if 'synced img time:' in line:
                parts = line.strip().split(',')
                synced_img_time = int(parts[2].split(':')[1].strip())
                synced_crop_time = int(parts[1].split(':')[1].strip())
                
                # Match image timestamp
                if synced_img_time == image_timestamp:
                    return synced_crop_time
    

def find_car_data_from_detection(crop_log_path, synced_crop_time):
    """Find car lat/lon/heading from detection record using synced crop time """
    with open(crop_log_path, 'r') as f:
        for line in f:
            # Parse line format
            if 'RTK gen_time:' in line:
                # Extract RTK gen_time
                parts = line.strip().split(',')
                rtk_gen_time = int(parts[2].split(':')[1].strip())
                
                # Match with synced crop time
                if rtk_gen_time == synced_crop_time:
                    # Extract car_lat, car_lon, heading
                    car_lat = float(parts[5].split(':')[1].strip())
                    car_lon = float(parts[6].split(':')[1].strip())
                    heading = float(parts[7].split(':')[1].strip())
                    
                    return car_lat, car_lon, heading
    

if __name__ == "__main__":

    # Read crack location from file
    crack_lat, crack_long = read_crack_location(crack_location_path)
    print(f"Loaded crack location - Lat: {crack_lat}, Lon: {crack_long}")

    # Extract timestamp from target image
    image_timestamp = extract_timestamp_from_image(target_image_path)
    print(f"Image timestamp: {image_timestamp}")

    # Find synced crop time from sync report
    synced_crop_time = find_synced_crop_time(sync_report_path, image_timestamp)
    print(f"Synced crop time: {synced_crop_time}")

    # Find car data from detection record (synced_crop_time matches RTK gen_time)
    car_lat, car_long, car_heading = find_car_data_from_detection(crop_log_path, synced_crop_time)
    print(f"Car position - Lat: {car_lat}, Lon: {car_long}, Heading: {car_heading}")

    # Find the YOLO box coordinates
    YOLO_box = Yolo_gt_HX(target_image_path, confidence_threshold=0.5)[0]


    # whatever is fixed in the data
    def tune_pitch_yaw(pitch, yaw):
        model = Cameras_HX(config_path, yaw, pitch)
        width = model.image_width
        height = model.image_height
        YOLO_center = ((YOLO_box[0] + YOLO_box[2]) / 2, (YOLO_box[1] + YOLO_box[3]) / 2)

        # whatever need to confirm with offline data
        x_enu, y_enu = model.gps_to_local(crack_lat, crack_long, car_lat, car_long)
        crack_car = model.enu_to_car_frame([x_enu, y_enu], car_heading)
        # print("Crack in car frame (Forward, Left):", crack_car)
        crack_camera = model.vehicle_to_camera_frame(crack_car)
        # print("Crack in camera frame (Right, Down, Forward):", crack_camera)
        u,v = model.project_to_pixel(crack_camera)
        # print("Projected pixel coordinates (u,v):", (u,v))
        x1, x2, y1, y2 = get_box(width, height, u, v, box_size=512)
        # print("Crop box (x1, x2, y1, y2):", (x1, x2, y1, y2))
        # crack_back_vehicle = model.pixel_to_vehicle_frame((u,v))
        # print("Back-projected crack in vehicle frame (Forward, Left, Up):", crack_back_vehicle)

        # whatever new need to be calculated 
        CROP_box = (x1, y1, x2, y2)
        CROP_center = ((x1 + x2) / 2, (y1 + y2) / 2)
        edge_coord = edge_projection(CROP_center, YOLO_center, 512, 512)
        # print("Edge coordinates (x_edge, y_edge):", edge_coord)
        score = finding_the_scores(YOLO_box, CROP_box, edge_coord)
        # print("Overall score:", score)
        return score

    # Define the range for pitch and yaw
    pitch_range = np.arange(-20, 11, 1)
    yaw_range = np.arange(-10, 11, 1)

    score_matrix = np.zeros((len(pitch_range), len(yaw_range)))
    # Brute-force search
    for i, pitch in enumerate(pitch_range):
        for j, yaw in enumerate(yaw_range):
            print(f"Testing pitch: {pitch}, yaw: {yaw}")
            score_matrix[i, j] = tune_pitch_yaw(pitch, yaw)

  
    max_score = np.max(score_matrix)
    max_indices = np.unravel_index(np.argmax(score_matrix, axis=None), score_matrix.shape)
    best_pitch = pitch_range[max_indices[0]]
    best_yaw = yaw_range[max_indices[1]]

    # Plotting the heatmap
    fig, ax = plt.subplots(figsize=(12, 10))
    cax = ax.imshow(score_matrix, cmap='viridis', interpolation='nearest', origin='lower',
                    extent=[yaw_range.min(), yaw_range.max(), pitch_range.min(), pitch_range.max()])
    
    # Adding a color bar
    fig.colorbar(cax, label='Score')

    # Setting labels and title
    ax.set_xlabel('Yaw (degrees)')
    ax.set_ylabel('Pitch (degrees)')
    ax.set_title('Score Heatmap')

    # Annotate the best score
    ax.text(best_yaw, best_pitch, f'Best\nScore: {max_score:.2f}',
            ha='center', va='center', color='white', fontsize=12,
            bbox=dict(boxstyle='round,pad=0.5', fc='red', alpha=0.5))
    
    print(f"Best score: {max_score:.2f} at pitch: {best_pitch}, yaw: {best_yaw}")

    # Show and save plot
    plt.savefig('heatmap.png')
    plt.show()

