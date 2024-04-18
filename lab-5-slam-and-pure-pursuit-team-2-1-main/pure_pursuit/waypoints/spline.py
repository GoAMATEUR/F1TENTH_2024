import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

# load four courners
points = np.loadtxt("/home/luyijie/f1tenth_ws/lab-5-slam-and-pure-pursuit-team-2-1/pure_pursuit/waypoints/wp_corner.csv", delimiter=",", dtype=float)
x = points[:, 0]
y = points[:, 1]

# interpolate
# tck, u = splprep([x, y], s=0)
# interpolated_points = np.array(splev(u, tck)).T

# create points between each points
# def generate_points(start, end, num_points):
#     return np.linspace(start, end, num_points)
def generate_points(start, end, dist):
    num_points = int(np.linalg.norm(end - start) / dist)
    return np.array([np.linspace(start[0], end[0], num_points), np.linspace(start[1], end[1], num_points)]).T

num_points_per_line = 10
dist = 0.5
# data = np.vstack([
#     generate_points(interpolated_points[i],
#                     interpolated_points[(i+1)%len(interpolated_points)], 
#                     dist)
#     for i in range(len(interpolated_points))
# ])
data = []
for i in range(len(points)-1):
    data += generate_points(points[i], points[i+1], dist).tolist()[1:]
data = np.array(data)

# visualize
fig, ax = plt.subplots()
ax.plot(x, y, 'ro')
ax.plot(data[:, 0], data[:, 1], 'bo')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
plt.show()

# save as csv
np.savetxt("/home/luyijie/f1tenth_ws/lab-5-slam-and-pure-pursuit-team-2-1/pure_pursuit/waypoints/wp_interpolated.csv", 
           data, 
           delimiter=",")