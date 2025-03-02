import matplotlib.pyplot as plt

# Read the trajectory data from the file.
times = []
x_vals = []
y_vals = []
z_vals = []

with open("cmake-build-debug/trajectory.txt", "r") as f:
    # Skip header line.
    next(f)
    for line in f:
        parts = line.strip().split()
        if len(parts) == 4:
            time, x, y, z = map(float, parts)
            times.append(time)
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)

# Create a 3D plot.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_vals, y_vals, z_vals, label="A320 Trajectory", marker="o", markersize=2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Simulated A320 Flight Path")
ax.legend()
plt.show()

