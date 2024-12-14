import sys
import lcm
import matplotlib.pyplot as plt
from exlcm import example_t # type: ignore

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)
    
log = lcm.EventLog(sys.argv[1], "r")

timestamps = []
x_positions = []
y_positions = []
orientations = []
lin_vel = []
rot_vel = []

for event in log:
    if event.channel == "MBOT_ODOMETRY":
        msg = example_t.decode(event.data)
        
        timestamps.append(msg.utime) 
        x_positions.append(msg.x)
        y_positions.append(msg.y)
        orientations.append(msg.theta)

        print("Message:")
        print(" timestamp = %s" % str(msg.utime))
        print(" position = %s" % str(msg.x))
        print(" orientation = %s" % str(msg.y))
        print(" ranges: %s" % str(msg.theta))
        print("")
        
plt.figure(figsize=(10, 6))
plt.plot(x_positions, y_positions, label="Position (x, y)")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Position Trajectory")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig("../media/trajectory_drive_square.png")

plt.figure(figsize=(10, 6))
plt.plot(timestamps, orientations, label="Orientation")
plt.xlabel("Time (s)")
plt.ylabel("Orientation (rad)")
plt.title("Orientation over Time")
plt.legend()
plt.grid(True)
plt.savefig("../media/orientation_drive_square.png")
