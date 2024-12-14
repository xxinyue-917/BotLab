import matplotlib.pyplot as plt

# Read cone positions from file
cone_positions = {'red': None, 'green': None, 'blue': None}
with open('cone_info.txt', 'r') as file:
    for line in file:
        color, coords = line.split(':')
        x_str, y_str = coords.strip().split(',')
        x = float(x_str.split('=')[1])
        y = float(y_str.split('=')[1])
        cone_positions[color.strip()] = (x, y)

# Plot the cone positions
for color, positions in cone_positions.items():
    if positions:
        x_coords, y_coords = [positions[0]], [positions[1]]
        plt.scatter(x_coords, y_coords, color=color, marker='o')

plt.title('Cone Positions')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
# Save the plot to the media folder
plt.savefig('./media/cone_positions')