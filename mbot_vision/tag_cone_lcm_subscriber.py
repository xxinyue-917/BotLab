import lcm
from mbot_lcm_msgs.mbot_cone_array_t import mbot_cone_array_t
from mbot_lcm_msgs.mbot_apriltag_array_t import mbot_apriltag_array_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
from math import cos
from math import sin

"""
This scripts subscribe to the MBOT_CONE_ARRAY and  MBOT_APRILTAG_ARRAY
"""
# Define the Cone class
class Cone:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
# Initialize the ConeList
cone = Cone(0, 0)
coneList = {"red": cone, "green": cone, "blue": cone}
with open("./cone_info.txt", "w") as file:
# with open("../mbot_autonomy/src/planning/cone_info.txt", "w") as file:
    for color, cone in coneList.items():
        file.write(f"{color}: x={cone.x:.2f}, y={cone.y:.2f}\n")
        
# Initialize Slam Pose
slamPose = pose2D_t()
slamPose.x = 0
slamPose.y = 0
slamPose.theta = 0

def cone_callback(channel, data):
    msg = mbot_cone_array_t.decode(data)
    if msg.array_size == 0:
        # print("No Cone Deteced")
        return
    else:
        for detection in msg.detections:
            name = detection.name
            x = detection.pose.x/1000
            y = detection.pose.z/1000
            if name == "red_cone":
                x = slamPose.x + (cos(slamPose.theta) * x - sin(slamPose.theta) * y)
                y = slamPose.y - (sin(slamPose.theta) * x + cos(slamPose.theta) * y)
                red = Cone(x, y)
                coneList["red"] = red
            elif name == "green_cone":
                x = slamPose.x + (cos(slamPose.theta) * x - sin(slamPose.theta) * y)
                y = slamPose.y - (sin(slamPose.theta) * x + cos(slamPose.theta) * y)
                green = Cone(x, y)
                coneList["green"] = green
            elif name == "blue_cone":
                x = slamPose.x + (cos(slamPose.theta) * x - sin(slamPose.theta) * y)
                y = slamPose.y - (sin(slamPose.theta) * x + cos(slamPose.theta) * y)
                blue = Cone(x, y)
                coneList["blue"] = blue
            # pos_text = f"{name}: x={x:.2f}, z={y:.2f}"
            # print(pos_text)
            # pos_text = f"SLAM Pose: x={slamPose.x:.2f}, y={slamPose.y:.2f}"
            # print(pos_text)
            with open("./cone_info.txt", "w") as file:
            # with open("../mbot_autonomy/src/planning/cone_info.txt", "w") as file:
                for color, cone in coneList.items():
                    file.write(f"{color}: x={cone.x:.2f}, y={cone.y:.2f}\n")

def apriltag_callback(channel, data):
    msg = mbot_apriltag_array_t.decode(data)
    if msg.array_size == 0:
        # print("No Tag Detected")
        return
    else:
        for detection in msg.detections:
            tag_id = detection.tag_id
            [roll, pitch, yaw] = detection.pose.angles_rpy
            [qx, qy, qz, qw] = detection.pose.angles_quat
            pos_text = f"Tag ID {tag_id}: x={detection.pose.x:.2f}, y={detection.pose.y:.2f}, z={detection.pose.z:.2f} "
            euler_text = f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
            # print(pos_text+euler_text)
            
def slam_pose_callback(channel, data):
    msg = pose2D_t.decode(data)
    slamPose.x = msg.x
    slamPose.y = msg.y
    slamPose.theta = msg.theta
    pos_text = f"SLAM Pose: x={slamPose.x:.2f}, y={slamPose.y:.2f}"
    print(pos_text)


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
lc.subscribe("MBOT_CONE_ARRAY", cone_callback)
lc.subscribe("MBOT_APRILTAG_ARRAY", apriltag_callback)
lc.subscribe("SLAM_POSE", slam_pose_callback)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass