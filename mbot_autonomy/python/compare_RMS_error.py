import numpy as np

def parse_log(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    data = []
    for line in lines:
        if 'timestamp' in line:
            timestamp = int(line.split('=')[1].strip())
        elif 'x' in line:
            x = float(line.split('=')[1].strip())
        elif 'y' in line:
            y = float(line.split('=')[1].strip())
        elif 'theta' in line:
            theta = float(line.split('=')[1].strip())
            data.append((timestamp, x, y, theta))
    
    return data

def calculate_rms_error(data1, data2):
    # Align data based on timestamps
    timestamps1 = {entry[0] for entry in data1}
    timestamps2 = {entry[0] for entry in data2}
    common_timestamps = timestamps1.intersection(timestamps2)
    
    if not common_timestamps:
        raise ValueError("No common timestamps found between the two datasets.")
    
    lin_errors = []
    rot_errors = []
    for timestamp in common_timestamps:
        entry1 = next(entry for entry in data1 if entry[0] == timestamp)
        entry2 = next(entry for entry in data2 if entry[0] == timestamp)
        
        x_error = (entry1[1] - entry2[1]) ** 2
        y_error = (entry1[2] - entry2[2]) ** 2
        theta_error = (entry1[3] - entry2[3]) ** 2
        
        lin_error = x_error + y_error
        lin_errors.append(lin_error)
        rot_errors.append(theta_error)
    
    rms_error_lin = np.sqrt(np.mean(lin_errors))
    rot_errors_lin = np.sqrt(np.mean(rot_errors))
    return rms_error_lin, rot_errors_lin

if __name__ == "__main__":
    ground_truth_file = 'slam_pose_log_messages.txt'
    slam_pose_file = 'slam_pose_messages.txt'
    
    ground_truth_data = parse_log(ground_truth_file)
    slam_pose_data = parse_log(slam_pose_file)
    
    rms_error_lin, rms_error_rot = calculate_rms_error(ground_truth_data, slam_pose_data)
    print(f"RMS Linear Error: {rms_error_lin}")
    print(f"RMS Rotational Error: {rms_error_rot}")