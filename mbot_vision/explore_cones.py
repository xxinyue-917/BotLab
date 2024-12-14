import lcm
from mbot_lcm_msgs.pose2D_t import pose2D_t
from mbot_lcm_msgs.path2D_t import path2D_t


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
# Read the cone info from the file
def read_cone_info(file_path):
    cone_info = {'red': None, 'green': None, 'blue': None}
    with open(file_path, 'r') as file:
        for line in file:
            color, coords = line.split(':')
            x_str, y_str = coords.strip().split(',')
            x = float(x_str.split('=')[1])
            y = float(y_str.split('=')[1])
            cone_info[color.strip()] = (x, y)
    return cone_info

cone_info = read_cone_info('./cone_info.txt')
# print(cone_info)

# Define the target cones in pose2D_t format
target_red = pose2D_t()
target_red.x = cone_info['red'][0]
target_red.y = cone_info['red'][1]

target_green = pose2D_t()
target_green.x = cone_info['green'][0]
target_green.y = cone_info['green'][1]

target_blue = pose2D_t()
target_blue.x = cone_info['blue'][0]
target_blue.y = cone_info['blue'][1]

# Pubilsh the position of red
lcmInstance.publish(CONTROLLER_PATH_CHANNEL, currentPath_);







# lc.subscribe("MBOT_CONE_ARRAY", cone_callback)
# lc.subscribe("MBOT_APRILTAG_ARRAY", apriltag_callback)
# lc.subscribe("SLAM_POSE", slam_pose_callback)